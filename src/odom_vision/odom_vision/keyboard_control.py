#!/usr/bin/env python3
import curses
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def quat_to_yaw(q):
    """Extract yaw (radians) from quaternion in ENU frame."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, 10)

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 2.0
        self.current_yaw = 0.0   # drone heading in world frame (radians)
        self.initialized = False
        self.step = 1.0  # meters per key press

        self.create_timer(0.05, self._publish)  # 20 Hz

    def _pose_cb(self, msg):
        self.current_yaw = quat_to_yaw(msg.pose.orientation)
        if not self.initialized:
            self.target_x = msg.pose.position.x
            self.target_y = msg.pose.position.y
            self.target_z = msg.pose.position.z
            self.initialized = True

    def _publish(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = self.target_z
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)

    def move(self, forward, right):
        """ forward/right in body frame (meters)."""
        yaw = self.current_yaw
        self.target_x += forward * math.cos(yaw) - right * math.sin(yaw)
        self.target_y += forward * math.sin(yaw) + right * math.cos(yaw)


def main():
    rclpy.init()
    node = KeyboardController()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    def curses_main(stdscr):
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(False)
        stdscr.clear()

        stdscr.addstr(0, 0, '=== Keyboard Position Control ===')
        stdscr.addstr(1, 0, 'Arrow keys: move 1m relative to drone heading | Q: quit')
        stdscr.addstr(2, 0, 'UP=Forward  DOWN=Back  LEFT=Strafe Left  RIGHT=Strafe Right')
        stdscr.addstr(3, 0, '')

        while True:
            stdscr.addstr(4, 0, f'Target -> X: {node.target_x:7.2f} m   Y: {node.target_y:7.2f} m   Z: {node.target_z:7.2f} m   ')
            stdscr.addstr(5, 0, f'Heading -> {math.degrees(node.current_yaw):6.1f} deg   {"Initialized" if node.initialized else "Waiting for pose..."}          ')
            stdscr.refresh()

            key = stdscr.getch()

            if key == curses.KEY_UP:
                node.move(node.step, 0)       # forward
            elif key == curses.KEY_DOWN:
                node.move(-node.step, 0)      # backward
            elif key == curses.KEY_LEFT:
                node.move(0, node.step)      # strafe left
            elif key == curses.KEY_RIGHT:
                node.move(0, -node.step)       # strafe right
            elif key in (ord('q'), ord('Q')):
                break

    try:
        curses.wrapper(curses_main)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
