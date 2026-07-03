#!/usr/bin/env python3
"""
Persistent interactive publisher for /move_command.

Unlike `ros2 topic pub --once`, this keeps the publisher alive between
commands so DDS discovery only happens ONCE at startup. After that,
every command is sent instantly.

Usage:
    ros2 run odom_vision send_move

Interactive commands (type at the prompt):
    5 0         → forward 5m, left 0m
    3 2         → forward 3m, left 2m
    -1 0        → backward 1m
    0 -2        → right 2m
    stop        → publish to /cancel_goal (hover in place)
    q / quit    → exit
"""

import sys
import threading

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SendMove(Node):
    def __init__(self):
        super().__init__("send_move")
        self.move_pub = self.create_publisher(Point, "/move_command", 10)
        self.cancel_pub = self.create_publisher(String, "/cancel_goal", 10)
        self.get_logger().info(
            "send_move ready. Waiting briefly for subscriber discovery..."
        )

    def send_move(self, forward: float, left: float) -> None:
        msg = Point()
        msg.x = forward
        msg.y = left
        msg.z = 0.0
        self.move_pub.publish(msg)
        self.get_logger().info(f"Sent /move_command: forward={forward}, left={left}")

    def send_cancel(self) -> None:
        msg = String()
        msg.data = "stop"
        self.cancel_pub.publish(msg)
        self.get_logger().info("Sent /cancel_goal")


def repl(node: SendMove) -> None:
    print()
    print("=" * 60)
    print("  send_move — interactive /move_command publisher")
    print("=" * 60)
    print("  Commands:")
    print("    <forward> <left>   e.g. '5 0' → 5m forward")
    print("    stop               → cancel current goal, hover")
    print("    q / quit           → exit")
    print("=" * 60)
    print()

    while rclpy.ok():
        try:
            line = input("move> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        if line in ("q", "quit", "exit"):
            break

        if line == "stop":
            node.send_cancel()
            continue

        parts = line.split()
        if len(parts) != 2:
            print("  ! expected: <forward> <left>   (e.g. '5 0')")
            continue

        try:
            forward = float(parts[0])
            left = float(parts[1])
        except ValueError:
            print("  ! values must be numbers")
            continue

        node.send_move(forward, left)


def main(args=None):
    rclpy.init(args=args)
    node = SendMove()

    # Spin in background so publishers stay discoverable
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        repl(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
