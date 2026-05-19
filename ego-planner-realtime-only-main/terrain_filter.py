#!/usr/bin/env python3
"""
地形过滤节点：
- 输入: /drone_0_cloud
- 输出: /drone_0_cloud_filtered
- 调试输出:
  - /drone_0_cloud_removed
  - /drone_0_cloud_front_guard

策略：
1. 使用已知赛场双侧坡面的世界坐标范围，剔除贴近坡面平面的点
2. 根据真实 /drone_0_odom，在机器人正前方为近距离大障碍生成一层保守的遮挡保护墙

第二步的目的不是“精确重建未知障碍”，而是避免规划器把未完全看清的近距遮挡区
过早当成可通行空域，从而把局部轨迹拉到机器人前方很远的位置。
"""

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


SCENE_X_OFFSET = 0.4


@dataclass(frozen=True)
class RampPlane:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    base_x: float
    slope_x: float
    z0: float = 0.0

    def plane_z(self, x: np.ndarray) -> np.ndarray:
        return self.z0 + self.slope_x * (x - self.base_x)


KNOWN_RAMPS = (
    RampPlane(
        name="blue_side_ramp",
        x_min=3.3 + SCENE_X_OFFSET,
        x_max=4.8 + SCENE_X_OFFSET,
        y_min=-6.05,
        y_max=-4.50,
        z_min=-0.05,
        z_max=0.45,
        base_x=3.3 + SCENE_X_OFFSET,
        slope_x=0.4 / 1.5,
    ),
    RampPlane(
        name="red_side_ramp",
        x_min=3.3 + SCENE_X_OFFSET,
        x_max=4.8 + SCENE_X_OFFSET,
        y_min=4.50,
        y_max=6.05,
        z_min=-0.05,
        z_max=0.45,
        base_x=3.3 + SCENE_X_OFFSET,
        slope_x=0.4 / 1.5,
    ),
)


class TerrainFilterNode(Node):
    def __init__(self):
        super().__init__("terrain_filter_node")

        self.remove_eps = 0.06
        self.roi_margin = 0.05
        self.min_fit_points = 30
        self.enable_flat_ground_filter = True
        self.flat_ground_z_min = -0.12
        self.flat_ground_z_max = 0.10
        self.enable_front_guard = True
        self.front_guard_min_points = 60
        self.front_guard_min_forward = 0.55
        self.front_guard_max_forward = 1.40
        self.front_guard_half_width = 0.75
        self.front_guard_face_window = 0.12
        self.front_guard_extra_width = 0.10
        self.front_guard_min_span = 0.28
        self.front_guard_depth = 0.22
        self.front_guard_slice_step = 0.20
        self.front_guard_lateral_step = 0.12
        self.front_guard_z_levels = np.array([0.08, 0.28, 0.48], dtype=np.float32)
        self.front_guard_min_rel_z = -0.20
        self.front_guard_max_rel_z = 0.85

        self.robot_position = None
        self.robot_yaw = 0.0
        self._last_log_ns = 0

        self.odom_sub = self.create_subscription(
            Odometry, "/drone_0_odom", self.odom_callback, 10
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/drone_0_cloud", self.cloud_callback, 10
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/drone_0_cloud_filtered", 10
        )
        self.removed_pub = self.create_publisher(
            PointCloud2, "/drone_0_cloud_removed", 10
        )
        self.guard_pub = self.create_publisher(
            PointCloud2, "/drone_0_cloud_front_guard", 10
        )

        self.get_logger().info("terrain_filter 已启动")
        self.get_logger().info("  里程计: /drone_0_odom")
        self.get_logger().info("  输入: /drone_0_cloud")
        self.get_logger().info("  输出: /drone_0_cloud_filtered")
        self.get_logger().info("  调试: /drone_0_cloud_removed")
        self.get_logger().info("  调试: /drone_0_cloud_front_guard")
        for ramp in KNOWN_RAMPS:
            self.get_logger().info(
                f"  坡面 ROI {ramp.name}: "
                f"x=[{ramp.x_min:.2f}, {ramp.x_max:.2f}], "
                f"y=[{ramp.y_min:.2f}, {ramp.y_max:.2f}]"
            )

    def odom_callback(self, msg: Odometry):
        self.robot_position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=np.float32,
        )
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def cloud_callback(self, msg: PointCloud2):
        points = self.read_xyz_cloud(msg)
        if points.size == 0:
            empty = self.build_xyz_cloud(msg, points)
            self.cloud_pub.publish(empty)
            self.removed_pub.publish(empty)
            self.guard_pub.publish(empty)
            return

        remove_mask, debug_lines = self.build_remove_mask(points)
        filtered_points = points[~remove_mask]
        removed_points = points[remove_mask]
        guard_points, guard_debug = self.build_front_guard_points(filtered_points)
        guarded_points = (
            filtered_points
            if guard_points.shape[0] == 0
            else np.concatenate((filtered_points, guard_points), axis=0).astype(np.float32)
        )

        self.cloud_pub.publish(self.build_xyz_cloud(msg, guarded_points))
        self.removed_pub.publish(self.build_xyz_cloud(msg, removed_points))
        self.guard_pub.publish(self.build_xyz_cloud(msg, guard_points))

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns > 2_000_000_000:
            self._last_log_ns = now_ns
            removed_ratio = 100.0 * removed_points.shape[0] / max(points.shape[0], 1)
            debug_msg = " | ".join(debug_lines) if debug_lines else "no ramp points"
            self.get_logger().info(
                f"terrain_filter: input={points.shape[0]} "
                f"removed={removed_points.shape[0]} ({removed_ratio:.1f}%) "
                f"guard={guard_points.shape[0]} "
                f"output={guarded_points.shape[0]} | {debug_msg} | {guard_debug}"
            )

    def build_remove_mask(self, points: np.ndarray):
        remove_mask = np.zeros(points.shape[0], dtype=bool)
        debug_lines = []

        if self.enable_flat_ground_filter:
            flat_ground_mask = (
                (points[:, 2] >= self.flat_ground_z_min)
                & (points[:, 2] <= self.flat_ground_z_max)
            )
            remove_mask |= flat_ground_mask
            if np.count_nonzero(flat_ground_mask) > 0:
                debug_lines.append(
                    f"flat_ground={int(np.count_nonzero(flat_ground_mask))}"
                )

        for ramp in KNOWN_RAMPS:
            roi_mask = self.points_in_ramp_roi(points, ramp)
            roi_points = points[roi_mask]

            if roi_points.shape[0] >= self.min_fit_points:
                slope_deg = self.estimate_ramp_slope_deg(roi_points)
                if slope_deg is not None:
                    debug_lines.append(
                        f"{ramp.name}: roi={roi_points.shape[0]} slope={slope_deg:.1f}deg"
                    )

            if roi_points.shape[0] == 0:
                continue

            plane_z = ramp.plane_z(points[:, 0])
            near_plane = np.abs(points[:, 2] - plane_z) <= self.remove_eps
            remove_mask |= roi_mask & near_plane

        return remove_mask, debug_lines

    def build_front_guard_points(self, points: np.ndarray):
        if (
            not self.enable_front_guard
            or self.robot_position is None
            or points.shape[0] == 0
        ):
            return np.empty((0, 3), dtype=np.float32), "front_guard=off"

        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        dx = points[:, 0] - self.robot_position[0]
        dy = points[:, 1] - self.robot_position[1]
        rel_z = points[:, 2] - self.robot_position[2]

        forward = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy

        candidate_mask = (
            (forward >= self.front_guard_min_forward)
            & (forward <= self.front_guard_max_forward)
            & (np.abs(lateral) <= self.front_guard_half_width)
            & (rel_z >= self.front_guard_min_rel_z)
            & (rel_z <= self.front_guard_max_rel_z)
        )

        candidate_count = int(np.count_nonzero(candidate_mask))
        if candidate_count < self.front_guard_min_points:
            return np.empty((0, 3), dtype=np.float32), "front_guard=idle"

        forward_candidates = forward[candidate_mask]
        lateral_candidates = lateral[candidate_mask]

        face_forward = float(np.percentile(forward_candidates, 15.0))
        face_mask = candidate_mask & (forward <= face_forward + self.front_guard_face_window)
        face_lateral = lateral[face_mask]
        if face_lateral.size < max(8, self.front_guard_min_points // 3):
            face_lateral = lateral_candidates

        lat_min = float(np.percentile(face_lateral, 5.0)) - self.front_guard_extra_width
        lat_max = float(np.percentile(face_lateral, 95.0)) + self.front_guard_extra_width
        span = lat_max - lat_min
        if span < self.front_guard_min_span:
            center = 0.5 * (lat_min + lat_max)
            lat_min = center - self.front_guard_min_span / 2.0
            lat_max = center + self.front_guard_min_span / 2.0
            span = self.front_guard_min_span

        forward_start = max(self.front_guard_min_forward, face_forward)
        forward_end = min(self.front_guard_max_forward, forward_start + self.front_guard_depth)

        forward_slices = np.arange(
            forward_start,
            forward_end + 1e-6,
            self.front_guard_slice_step,
            dtype=np.float32,
        )
        lateral_samples = np.arange(
            lat_min,
            lat_max + 1e-6,
            self.front_guard_lateral_step,
            dtype=np.float32,
        )

        if forward_slices.size == 0 or lateral_samples.size == 0:
            return np.empty((0, 3), dtype=np.float32), "front_guard=idle"

        guard_points = np.empty(
            (
                forward_slices.size
                * lateral_samples.size
                * self.front_guard_z_levels.size,
                3,
            ),
            dtype=np.float32,
        )

        idx = 0
        for forward_sample in forward_slices:
            for lateral_sample in lateral_samples:
                world_x = (
                    self.robot_position[0]
                    + cos_yaw * forward_sample
                    - sin_yaw * lateral_sample
                )
                world_y = (
                    self.robot_position[1]
                    + sin_yaw * forward_sample
                    + cos_yaw * lateral_sample
                )
                for z_level in self.front_guard_z_levels:
                    guard_points[idx, 0] = world_x
                    guard_points[idx, 1] = world_y
                    guard_points[idx, 2] = self.robot_position[2] + z_level
                    idx += 1

        debug = (
            f"front_guard=on pts={candidate_count} "
            f"face={face_forward:.2f}m span={span:.2f}m depth={forward_end - forward_start:.2f}m"
        )
        return guard_points[:idx], debug

    def points_in_ramp_roi(self, points: np.ndarray, ramp: RampPlane) -> np.ndarray:
        return (
            (points[:, 0] >= ramp.x_min - self.roi_margin)
            & (points[:, 0] <= ramp.x_max + self.roi_margin)
            & (points[:, 1] >= ramp.y_min - self.roi_margin)
            & (points[:, 1] <= ramp.y_max + self.roi_margin)
            & (points[:, 2] >= ramp.z_min)
            & (points[:, 2] <= ramp.z_max)
        )

    def estimate_ramp_slope_deg(self, points: np.ndarray):
        a = np.column_stack((points[:, 0], points[:, 1], np.ones(points.shape[0])))
        b = points[:, 2]
        try:
            coeffs, *_ = np.linalg.lstsq(a, b, rcond=None)
        except np.linalg.LinAlgError:
            return None

        slope = math.sqrt(coeffs[0] ** 2 + coeffs[1] ** 2)
        return math.degrees(math.atan(slope))

    def read_xyz_cloud(self, msg: PointCloud2) -> np.ndarray:
        point_count = msg.width * msg.height
        if point_count == 0:
            return np.empty((0, 3), dtype=np.float32)

        cols = msg.point_step // 4
        if cols < 3:
            return np.empty((0, 3), dtype=np.float32)

        raw = np.frombuffer(msg.data, dtype=np.float32)
        if raw.size < point_count * cols:
            return np.empty((0, 3), dtype=np.float32)

        raw = raw.reshape(point_count, cols)
        points = raw[:, :3].copy()
        finite_mask = np.isfinite(points).all(axis=1)
        return points[finite_mask]

    def build_xyz_cloud(self, src_msg: PointCloud2, points: np.ndarray) -> PointCloud2:
        new_msg = PointCloud2()
        new_msg.header.stamp = src_msg.header.stamp
        new_msg.header.frame_id = src_msg.header.frame_id
        new_msg.height = 1
        new_msg.width = int(points.shape[0])
        new_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        new_msg.is_bigendian = False
        new_msg.point_step = 12
        new_msg.row_step = 12 * new_msg.width
        new_msg.data = points.astype(np.float32).tobytes()
        new_msg.is_dense = True
        return new_msg


def main(args=None):
    rclpy.init(args=args)
    node = TerrainFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
