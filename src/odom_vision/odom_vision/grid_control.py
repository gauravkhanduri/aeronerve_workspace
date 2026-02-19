#!/usr/bin/env python3
import math
import threading

import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


GRID_SIZE_M = 10        # total grid span (meters)
WINDOW_PX = 800       # window size (pixels)
SCALE = WINDOW_PX / GRID_SIZE_M   # 80 px/m
GRID_STEP_M = 1         # grid line every N meters
LABEL_STEP_M = 1         # label every N meters


def quat_to_yaw(q):
    """Extract yaw (radians) from quaternion in ENU frame."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def w2p(wx, wy, ox, oy):
    """World coords (m) → pixel coords. Origin at window centre."""
    px = int((wx - ox) * SCALE + WINDOW_PX / 2)
    py = int(-(wy - oy) * SCALE + WINDOW_PX / 2)   # screen Y is flipped
    return px, py


def p2w(px, py, ox, oy):
    """Pixel coords → world coords (m)."""
    wx = (px - WINDOW_PX / 2) / SCALE + ox
    wy = -(py - WINDOW_PX / 2) / SCALE + oy
    return wx, wy


class GridController(Node):

    def __init__(self):
        super().__init__('grid_controller')

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, mavros_qos)

        self.origin_x = 0.0
        self.origin_y = 0.0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 2.0
        self.drone_yaw = 0.0   # radians, ENU frame

        self.goal_x = None
        self.goal_y = None
        self.goal_z = 2.0

        self.initialized = False

    def _pose_cb(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_yaw = quat_to_yaw(msg.pose.orientation)
        if not self.initialized:
            self.origin_x = msg.pose.position.x
            self.origin_y = msg.pose.position.y
            self.drone_z = msg.pose.position.z
            self.goal_x = self.origin_x
            self.goal_y = self.origin_y
            self.goal_z = self.drone_z
            self.initialized = True

    def _publish(self):
        if not self.initialized or self.goal_x is None:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        pose.pose.position.z = self.goal_z
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)

    def set_goal(self, wx, wy):
        self.goal_x = wx
        self.goal_y = wy


# ── drawing helpers ──────────────────────────────────────────────────────────

def draw_grid(img, ox, oy):
    half = GRID_SIZE_M // 2
    for m in range(-half, half + 1, GRID_STEP_M):
        is_major = (m % LABEL_STEP_M == 0)
        color = (65, 65, 65) if is_major else (40, 40, 40)
        thick = 1

        # vertical lines (fixed world-X)
        px, _ = w2p(ox + m, oy, ox, oy)
        if 0 <= px < WINDOW_PX:
            cv2.line(img, (px, 0), (px, WINDOW_PX), color, thick)
            if is_major:
                cv2.putText(img, f'{m}', (px + 2, WINDOW_PX - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (90, 90, 90), 1)

        # horizontal lines (fixed world-Y)
        _, py = w2p(ox, oy + m, ox, oy)
        if 0 <= py < WINDOW_PX:
            cv2.line(img, (0, py), (WINDOW_PX, py), color, thick)
            if is_major:
                cv2.putText(img, f'{m}', (2, py - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (90, 90, 90), 1)

    # brighter axis lines through origin
    ax, ay = w2p(ox, oy, ox, oy)
    cv2.line(img, (ax, 0), (ax, WINDOW_PX), (80, 80, 80), 2)
    cv2.line(img, (0, ay), (WINDOW_PX, ay), (80, 80, 80), 2)

    # axis labels
    cv2.putText(img, 'X (East)', (WINDOW_PX - 70, ay - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
    cv2.putText(img, 'Y (North)', (ax + 5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)


# ── mouse callback ────────────────────────────────────────────────────────────

def on_mouse(event, px, py, flags, node):
    if event == cv2.EVENT_LBUTTONDOWN and node.initialized:
        wx, wy = p2w(px, py, node.origin_x, node.origin_y)
        node.set_goal(wx, wy)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = GridController()

    # ROS spin in background; OpenCV requires main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 20 Hz publish timer driven from the OpenCV loop
    WIN = 'Grid Controller  |  click = set goal  |  Q = quit'
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse, node)

    while True:
        img = np.full((WINDOW_PX, WINDOW_PX, 3), 30, dtype=np.uint8)

        if not node.initialized:
            cv2.putText(img, 'Waiting for /mavros/local_position/pose ...',
                        (120, WINDOW_PX // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (180, 180, 180), 2)
        else:
            draw_grid(img, node.origin_x, node.origin_y)

            # origin marker (yellow diamond)
            ox, oy = w2p(node.origin_x, node.origin_y, node.origin_x, node.origin_y)
            cv2.drawMarker(img, (ox, oy), (0, 220, 220),
                           cv2.MARKER_DIAMOND, 14, 2)
            cv2.putText(img, 'Start', (ox + 6, oy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 220, 220), 1)

            # goal marker (red cross)
            if node.goal_x is not None:
                gx, gy = w2p(node.goal_x, node.goal_y, node.origin_x, node.origin_y)
                cv2.drawMarker(img, (gx, gy), (0, 0, 255),
                               cv2.MARKER_CROSS, 22, 2)
                gx_rel = node.goal_x - node.origin_x
                gy_rel = node.goal_y - node.origin_y
                label = f'Goal ({gx_rel:+.1f}, {gy_rel:+.1f}) m'
                cv2.putText(img, label, (gx + 6, gy - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 80, 255), 1)

            # drone dot + heading arrow (green)
            dx, dy = w2p(node.drone_x, node.drone_y, node.origin_x, node.origin_y)
            cv2.circle(img, (dx, dy), 8, (0, 255, 0), -1)
            cv2.circle(img, (dx, dy), 8, (255, 255, 255), 1)

            # heading arrow: 20px long in drone-forward direction
            arrow_len = 20
            yaw = node.drone_yaw
            ax = int(dx + arrow_len * math.cos(yaw))
            ay = int(dy - arrow_len * math.sin(yaw))   # screen Y is flipped
            cv2.arrowedLine(img, (dx, dy), (ax, ay), (0, 255, 0), 2, tipLength=0.4)

            heading_deg = math.degrees(yaw) % 360
            drone_label = (f'({node.drone_x - node.origin_x:+.1f}, '
                           f'{node.drone_y - node.origin_y:+.1f}) m  '
                           f'{heading_deg:.0f}°')
            cv2.putText(img, drone_label, (dx + 10, dy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)

            node._publish()   # publish setpoint at display rate (~20 Hz)

        # HUD
        cv2.putText(img, 'Left-click: set goal | Q: quit | grid = 1m',
                    (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow(WIN, img)
        key = cv2.waitKey(50)   # ~20 Hz
        if key in (ord('q'), ord('Q'), 27):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
