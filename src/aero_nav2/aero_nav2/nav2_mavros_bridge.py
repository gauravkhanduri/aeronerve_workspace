#!/usr/bin/env python3
"""
nav2_mavros_bridge.py
======================
Bridges Nav2 /cmd_vel → /mavros/setpoint_position/local (PoseStamped).

Pattern matches keyboard_control.py and astar_planner.py from this workspace,
which are confirmed working with ArduPilot + MAVROS:
  - Default QoS (depth=10) on all topics
  - No use_sim_time — create_timer uses wall clock, fires reliably
  - PoseStamped with frame_id='map' to /mavros/setpoint_position/local

Carrot-on-stick
---------------
  target_pos = current_drone_pos + lookahead_dist * normalize(cmd_vel_world)

ArduPilot always has a clear target lookahead_dist metres ahead.
When cmd_vel goes stale, the target freezes so the drone hovers.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class Nav2MavrosBridge(Node):

    def __init__(self):
        super().__init__('nav2_mavros_bridge')

        self.declare_parameter('lookahead_dist', 2.0)
        self._lookahead = self.get_parameter('lookahead_dist').value

        # State — mirrors keyboard_control.py pattern
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 3.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 3.0
        self.current_yaw = 0.0
        self.initialized = False

        # MAVROS publishes local_position/pose at BEST_EFFORT — must match
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, sensor_qos)

        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # Wall-clock timer — no sim_time param → never stalls
        self.create_timer(0.05, self._publish)   # 20 Hz

        self.get_logger().info(
            f'nav2_mavros_bridge ready  lookahead={self._lookahead} m')

    def _pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        if not self.initialized:
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.initialized = True
            self.get_logger().info(
                f'Hover position set: ({self.target_x:.2f}, '
                f'{self.target_y:.2f}, {self.target_z:.2f})')

    def _cmd_vel_cb(self, msg: Twist):
        if not self.initialized:
            return

        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        speed = math.sqrt(vx ** 2 + vy ** 2)
        if speed > 0.05:
            travel_time = self._lookahead / speed

            if abs(omega) > 0.01:
                # Arc motion: project lookahead along the arc in body frame.
                # DWB steers via angular.z — integrate the arc to find where
                # the drone should be heading after travelling lookahead_dist.
                dtheta = omega * travel_time
                r = speed / omega          # signed arc radius
                dx_body = r * math.sin(dtheta)
                dy_body = r * (1.0 - math.cos(dtheta))
            else:
                # Straight motion in body frame
                dx_body = vx * travel_time
                dy_body = vy * travel_time

            # Rotate body-frame offset → world frame using current yaw
            c = math.cos(self.current_yaw)
            s = math.sin(self.current_yaw)
            self.target_x = self.current_x + c * dx_body - s * dy_body
            self.target_y = self.current_y + s * dx_body + c * dy_body

        if abs(msg.linear.z) > 0.05:
            self.target_z = self.current_z + msg.linear.z * self._lookahead

    def _publish(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = self.target_z
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2MavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
