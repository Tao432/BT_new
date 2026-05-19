#!/usr/bin/env python3
"""
话题中继节点：将 ROBOCON MuJoCo 仿真的话题转换为 EGO-Planner 期望的格式。

旧版工作流兼容行为：
- 节点名保持为 topic_relay_node
- 启动日志保持旧格式
- 点云回调通过 manual_transform_and_publish() 处理
"""

import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import tf2_ros


def quat_to_rotation_matrix(q):
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵"""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class TopicRelayNode(Node):
    def __init__(
        self,
        input_cloud_topic: str,
        output_odom_topic: str,
        output_cloud_topic: str,
        odom_parent_frame: str,
        odom_child_frame: str,
        world_frame: str,
        publish_world_to_odom_tf: bool,
        cloud_frame_override: str,
    ):
        super().__init__('topic_relay_node')

        self.input_cloud_topic = input_cloud_topic
        self.output_odom_topic = output_odom_topic
        self.output_cloud_topic = output_cloud_topic
        self.odom_parent_frame = odom_parent_frame
        self.odom_child_frame = odom_child_frame
        self.world_frame = world_frame
        self.publish_world_to_odom_tf_enabled = publish_world_to_odom_tf
        self.cloud_frame_override = cloud_frame_override.strip()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf = (
            tf2_ros.StaticTransformBroadcaster(self)
            if self.publish_world_to_odom_tf_enabled
            else None
        )

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_cloud_topic, 10)

        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self.cloud_callback, 10)
        self.tf_timer = self.create_timer(0.02, self.tf_to_odom_callback)

        self._logged_odom = False
        self._logged_cloud = False
        self._last_missing_odom_tf_warn_ns = 0
        self._last_missing_cloud_tf_warn_ns = 0
        self._last_missing_cloud_frame_warn_ns = 0

        if self.publish_world_to_odom_tf_enabled:
            self.publish_world_to_odom_tf()

        self.get_logger().info('话题中继节点已启动 (带坐标变换)')
        self.get_logger().info(
            f'  TF {self.world_frame} -> {self.odom_child_frame} -> {self.output_odom_topic}')
        self.get_logger().info(
            f'  {self.input_cloud_topic} -> {self.output_cloud_topic} '
            f'({self.world_frame}<-lidar TF 变换)')
        if not self.publish_world_to_odom_tf_enabled:
            self.get_logger().info(
                f'  不发布静态 TF {self.world_frame} -> {self.odom_parent_frame}，'
                '将直接使用外部 TF 树')
        if self.cloud_frame_override:
            self.get_logger().info(
                f'  cloud_frame_override = {self.cloud_frame_override}')

    def publish_world_to_odom_tf(self):
        if self.static_tf is None:
            return
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self.world_frame
        st.child_frame_id = self.odom_parent_frame
        st.transform.translation.x = 0.0
        st.transform.translation.y = 0.0
        st.transform.translation.z = 0.0
        st.transform.rotation.w = 1.0
        self.static_tf.sendTransform(st)

    def throttled_warn(self, attr_name: str, message: str, interval_ns: int = 2_000_000_000):
        now_ns = self.get_clock().now().nanoseconds
        last_ns = getattr(self, attr_name)
        if now_ns - last_ns >= interval_ns:
            setattr(self, attr_name, now_ns)
            self.get_logger().warn(message)

    def tf_to_odom_callback(self):
        """从 TF 树提取 world->base 变换，发布为规划用 odom。"""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.world_frame, self.odom_child_frame, rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.throttled_warn(
                '_last_missing_odom_tf_warn_ns',
                f'等待 TF: {self.world_frame} -> {self.odom_child_frame}，'
                '请确认定位链路和 TF 树已经发布。')
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = trans.header.stamp
        odom_msg.header.frame_id = self.world_frame
        odom_msg.child_frame_id = self.odom_child_frame
        odom_msg.pose.pose.position.x = trans.transform.translation.x
        odom_msg.pose.pose.position.y = trans.transform.translation.y
        odom_msg.pose.pose.position.z = trans.transform.translation.z
        odom_msg.pose.pose.orientation = trans.transform.rotation

        try:
            self.odom_pub.publish(odom_msg)
        except Exception:
            return

        if not self._logged_odom:
            self._logged_odom = True
            p = trans.transform.translation
            self.get_logger().info(
                f'首次发布 odom: x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}')

    def cloud_callback(self, msg: PointCloud2):
        self.manual_transform_and_publish(msg)

    def manual_transform_and_publish(self, msg: PointCloud2):
        """将 lidar 局部点云手动变换到目标世界系后发布。"""
        source_frame = msg.header.frame_id or self.cloud_frame_override
        if not source_frame:
            self.throttled_warn(
                '_last_missing_cloud_frame_warn_ns',
                f'{self.input_cloud_topic} 的 frame_id 为空，且未设置 cloud_frame_override。')
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.world_frame, source_frame, rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.throttled_warn(
                '_last_missing_cloud_tf_warn_ns',
                f'等待 TF: {self.world_frame} -> {source_frame}，'
                '请确认外部已经发布 world/odom/base/lidar 之间的 TF 链。')
            return

        point_count = msg.width * msg.height
        if point_count == 0:
            return

        raw = np.frombuffer(msg.data, dtype=np.float32)
        cols = msg.point_step // 4
        if cols < 3 or raw.size < point_count * cols:
            return

        raw = raw.reshape(point_count, cols)
        pts_local = raw[:, :3].copy()

        t = trans.transform.translation
        r = trans.transform.rotation
        rot = quat_to_rotation_matrix([r.x, r.y, r.z, r.w])
        offset = np.array([t.x, t.y, t.z], dtype=np.float32)
        pts_world = (rot @ pts_local.T).T + offset

        new_msg = PointCloud2()
        new_msg.header.stamp = msg.header.stamp
        new_msg.header.frame_id = self.world_frame
        new_msg.height = 1
        new_msg.width = point_count
        new_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        new_msg.is_bigendian = False
        new_msg.point_step = 12
        new_msg.row_step = 12 * point_count
        new_msg.data = pts_world.astype(np.float32).tobytes()
        new_msg.is_dense = True

        try:
            self.cloud_pub.publish(new_msg)
        except Exception:
            return

        if not self._logged_cloud:
            self._logged_cloud = True
            self.get_logger().info(
                f'首次发布 cloud: {point_count} 个点, '
                f'x=[{pts_world[:, 0].min():.1f}, {pts_world[:, 0].max():.1f}]')


def str2bool(value: str) -> bool:
    value = value.strip().lower()
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise argparse.ArgumentTypeError(f'无法解析布尔值: {value}')


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description='将 3D 雷达点云 + TF 转成 EGO-Planner 需要的 /drone_0_odom 和 /drone_0_cloud'
    )
    parser.add_argument('--input-cloud-topic', default='/lidar_points')
    parser.add_argument('--output-odom-topic', default='/drone_0_odom')
    parser.add_argument('--output-cloud-topic', default='/drone_0_cloud')
    parser.add_argument('--odom-parent-frame', default='odom')
    parser.add_argument('--odom-child-frame', default='imu')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--publish-world-to-odom-tf', type=str2bool, default=True)
    parser.add_argument('--cloud-frame-override', default='')
    return parser


def main(args=None):
    cli_args = build_arg_parser().parse_args(args=args)
    rclpy.init(args=None)
    node = TopicRelayNode(
        input_cloud_topic=cli_args.input_cloud_topic,
        output_odom_topic=cli_args.output_odom_topic,
        output_cloud_topic=cli_args.output_cloud_topic,
        odom_parent_frame=cli_args.odom_parent_frame,
        odom_child_frame=cli_args.odom_child_frame,
        world_frame=cli_args.world_frame,
        publish_world_to_odom_tf=cli_args.publish_world_to_odom_tf,
        cloud_frame_override=cli_args.cloud_frame_override,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
