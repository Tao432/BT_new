#!/usr/bin/env python3
"""
桥接节点：将 ego-planner 的位置命令转换为速度命令
输入: /drone_0_planning/pos_cmd (quadrotor_msgs/PositionCommand)
输出: /cmd_vel (geometry_msgs/Twist)
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


def quat_to_rpy(q):
    """从四元数计算 roll / pitch / yaw。"""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PosToCmdVelBridge(Node):
    def __init__(
        self,
        current_odom_topic: str,
        target_odom_topic: str,
        terrain_mode_topic: str,
        terrain_mode_active_topic: str,
        cmd_topic: str,
    ):
        super().__init__('pos_to_cmdvel_bridge')

        self.current_odom_topic = current_odom_topic
        self.target_odom_topic = target_odom_topic
        self.terrain_mode_topic = terrain_mode_topic
        self.terrain_mode_active_topic = terrain_mode_active_topic
        self.cmd_topic = cmd_topic
        
        # 订阅当前位置（来自 topic_relay 发布的里程计）
        self.odom_sub = self.create_subscription(
            Odometry,
            self.current_odom_topic,
            self.odom_callback,
            10
        )
        
        # 订阅目标位置 (从 poscmd_2_odom 输出的 odometry)
        self.target_sub = self.create_subscription(
            Odometry,
            self.target_odom_topic,  # ego-planner 的 poscmd_2_odom 节点输出
            self.target_callback,
            10
        )

        # 手动覆盖地形模式：flat / ramp / auto
        self.terrain_mode_sub = self.create_subscription(
            String,
            self.terrain_mode_topic,
            self.terrain_mode_callback,
            10
        )

        # 发布速度命令到机器人
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.terrain_mode_pub = self.create_publisher(
            String, self.terrain_mode_active_topic, 10)

        # 平地控制参数
        self.flat_max_linear_vel = 0.5
        self.flat_max_angular_vel = 2.0
        self.flat_linear_gain = 1.5
        self.flat_angular_gain = 5.0
        self.flat_stop_turn_error = 0.3
        self.flat_slow_turn_error = 0.15

        # 上坡控制参数：更保守，避免坡上急转和速度过大
        self.ramp_max_linear_vel = 0.22
        self.ramp_max_angular_vel = 0.7
        self.ramp_linear_gain = 0.9
        self.ramp_angular_gain = 2.5
        self.ramp_stop_turn_error = 0.2
        self.ramp_slow_turn_error = 0.1

        # 地形模式切换阈值（基于 pitch，带迟滞）
        self.ramp_enter_pitch = math.radians(6.0)
        self.ramp_exit_pitch = math.radians(3.0)
        self.ramp_enter_cycles = 4
        self.ramp_exit_cycles = 8
        self.position_tolerance = 0.1  # 位置容差
        self.flat_tracking_window = 0.45
        self.ramp_tracking_window = 0.22
        self.target_debug_throttle = 0.5

        self.current_pos = None
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = None
        self.target_pos = None
        self.target_yaw = None
        self.last_target_clamp_log_time = 0.0
        self.manual_terrain_mode = 'auto'
        self.auto_terrain_mode = 'flat'
        self.active_terrain_mode = 'flat'
        self.ramp_detect_count = 0
        self.flat_detect_count = 0

        # 定时器：以 20Hz 控制频率
        self.control_timer = self.create_timer(0.05, self.control_callback)

        self.get_logger().info('位置到速度桥接节点已启动')
        self.get_logger().info(
            f'订阅: {self.target_odom_topic} (目标位置, 来自 poscmd_2_odom)')
        self.get_logger().info(
            f'订阅: {self.current_odom_topic} (当前位置, 来自 topic_relay/实体机)')
        self.get_logger().info(
            f'订阅: {self.terrain_mode_topic} (flat / ramp / auto 手动切换)')
        self.get_logger().info(
            f'发布: {self.terrain_mode_active_topic} (当前生效模式)')
        self.get_logger().info(
            f'发布: {self.cmd_topic} (给下位机的 geometry_msgs/Twist)')
        self.publish_active_mode(force_log=True)

    def odom_callback(self, msg: Odometry):
        """接收当前位置"""
        self.current_pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_roll, self.current_pitch, self.current_yaw = quat_to_rpy(q)
        self.update_auto_terrain_mode()

    def target_callback(self, msg: Odometry):
        """接收目标位置"""
        self.target_pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, self.target_yaw = quat_to_rpy(q)

    def terrain_mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        if mode not in ('flat', 'ramp', 'auto'):
            self.get_logger().warn(
                f'忽略非法 terrain_mode: {msg.data!r}，可用值: flat / ramp / auto')
            return

        self.manual_terrain_mode = mode
        self.publish_active_mode(force_log=True)

    def update_auto_terrain_mode(self):
        abs_pitch = abs(self.current_pitch)

        if abs_pitch >= self.ramp_enter_pitch:
            self.ramp_detect_count += 1
            self.flat_detect_count = 0
        elif abs_pitch <= self.ramp_exit_pitch:
            self.flat_detect_count += 1
            self.ramp_detect_count = 0
        else:
            self.ramp_detect_count = 0
            self.flat_detect_count = 0

        if self.auto_terrain_mode != 'ramp' and \
                self.ramp_detect_count >= self.ramp_enter_cycles:
            self.auto_terrain_mode = 'ramp'
            self.publish_active_mode(force_log=True)
        elif self.auto_terrain_mode != 'flat' and \
                self.flat_detect_count >= self.ramp_exit_cycles:
            self.auto_terrain_mode = 'flat'
            self.publish_active_mode(force_log=True)

    def publish_active_mode(self, force_log=False):
        previous_mode = self.active_terrain_mode
        if self.manual_terrain_mode == 'auto':
            self.active_terrain_mode = self.auto_terrain_mode
        else:
            self.active_terrain_mode = self.manual_terrain_mode

        msg = String()
        msg.data = self.active_terrain_mode
        self.terrain_mode_pub.publish(msg)

        if force_log or self.active_terrain_mode != previous_mode:
            pitch_deg = math.degrees(self.current_pitch)
            self.get_logger().info(
                f'地形模式切换 -> {self.active_terrain_mode} '
                f'(manual={self.manual_terrain_mode}, '
                f'auto={self.auto_terrain_mode}, pitch={pitch_deg:.1f}deg)')

    def get_control_params(self):
        if self.active_terrain_mode == 'ramp':
            return {
                'max_linear_vel': self.ramp_max_linear_vel,
                'max_angular_vel': self.ramp_max_angular_vel,
                'linear_gain': self.ramp_linear_gain,
                'angular_gain': self.ramp_angular_gain,
                'stop_turn_error': self.ramp_stop_turn_error,
                'slow_turn_error': self.ramp_slow_turn_error,
                'tracking_window': self.ramp_tracking_window,
            }

        return {
            'max_linear_vel': self.flat_max_linear_vel,
            'max_angular_vel': self.flat_max_angular_vel,
            'linear_gain': self.flat_linear_gain,
            'angular_gain': self.flat_angular_gain,
            'stop_turn_error': self.flat_stop_turn_error,
            'slow_turn_error': self.flat_slow_turn_error,
            'tracking_window': self.flat_tracking_window,
        }

    def control_callback(self):
        """控制循环：计算并发布速度命令"""
        if self.current_pos is None or self.target_pos is None:
            return

        params = self.get_control_params()

        # 计算位置误差。这里不能直接追 /odometry 的完整超前轨迹，
        # 否则规划器时间轴走得比实体机器人快时，桥接会把机器人拖着追远点。
        raw_dx = self.target_pos.x - self.current_pos.x
        raw_dy = self.target_pos.y - self.current_pos.y
        raw_distance = math.sqrt(raw_dx * raw_dx + raw_dy * raw_dy)

        tracking_window = params['tracking_window']
        if raw_distance > tracking_window and raw_distance > 1e-6:
            scale = tracking_window / raw_distance
            dx = raw_dx * scale
            dy = raw_dy * scale
            distance = tracking_window

            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_target_clamp_log_time > self.target_debug_throttle:
                self.last_target_clamp_log_time = now_sec
                self.get_logger().info(
                    f'局部追踪窗口生效: raw_dist={raw_distance:.2f} -> {tracking_window:.2f} m',
                    throttle_duration_sec=0.5
                )
        else:
            dx = raw_dx
            dy = raw_dy
            distance = raw_distance

        # 计算目标方向
        target_angle = math.atan2(dy, dx)

        # 计算角度误差 (归一化到 [-pi, pi])
        if self.current_yaw is not None:
            angle_error = target_angle - self.current_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        else:
            angle_error = 0.0

        # 比例控制器
        # 线速度：与距离成正比
        linear_vel = min(
            params['linear_gain'] * distance,
            params['max_linear_vel']
        )

        # 角速度：与角度误差成正比
        angular_vel = params['angular_gain'] * angle_error
        angular_vel = max(
            min(angular_vel, params['max_angular_vel']),
            -params['max_angular_vel']
        )

        # 如果角度误差很大，先原地转向
        if abs(angle_error) > params['stop_turn_error']:
            linear_vel = 0.0
            if self.active_terrain_mode == 'ramp':
                angular_vel *= 0.5
        elif abs(angle_error) > params['slow_turn_error']:
            linear_vel *= 0.4 if self.active_terrain_mode == 'ramp' else 0.5

        if self.active_terrain_mode == 'ramp':
            # 坡上进一步保守一点，随 pitch 增大自动降速
            pitch_scale = max(0.45, 1.0 - abs(self.current_pitch) / math.radians(20.0))
            linear_vel *= pitch_scale

        # 创建 Twist 消息
        cmd = Twist()

        # 如果距离目标很近，停止
        if distance < self.position_tolerance:
            linear_vel = 0.0
            angular_vel = 0.0
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
        else:
            # 在机器人坐标系下的速度（前进方向）
            cmd.linear.x = linear_vel
            cmd.linear.y = 0.0
            cmd.angular.z = angular_vel

        # 打印调试信息
        if abs(angle_error) > 0.1 or distance > 0.1:
             self.get_logger().info(
                 f'Mode: {self.active_terrain_mode}, '
                 f'Pitch: {math.degrees(self.current_pitch):.1f}deg, '
                 f'Dist: {distance:.2f}, RawDist: {raw_distance:.2f}, AngErr: {angle_error:.2f}, '
                 f'CmdLin: {linear_vel:.2f}, CmdAng: {angular_vel:.2f}',
                 throttle_duration_sec=0.5
             )

        # 发布命令
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    parser = argparse.ArgumentParser(
        description='将 EGO-Planner 的目标 odometry 转成可直接给下位机的 geometry_msgs/Twist')
    parser.add_argument('--current-odom-topic', default='/drone_0_odom')
    parser.add_argument('--target-odom-topic', default='/odometry')
    parser.add_argument('--terrain-mode-topic', default='/terrain_mode')
    parser.add_argument('--terrain-mode-active-topic', default='/terrain_mode_active')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    cli_args = parser.parse_args(args=args)

    rclpy.init(args=None)
    node = PosToCmdVelBridge(
        current_odom_topic=cli_args.current_odom_topic,
        target_odom_topic=cli_args.target_odom_topic,
        terrain_mode_topic=cli_args.terrain_mode_topic,
        terrain_mode_active_topic=cli_args.terrain_mode_active_topic,
        cmd_topic=cli_args.cmd_topic,
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



