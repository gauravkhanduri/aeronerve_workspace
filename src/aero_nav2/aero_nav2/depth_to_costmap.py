#!/usr/bin/env python3
"""
depth_to_costmap.py
====================
Converts the raw depth camera image to a PointCloud2 that Nav2's
VoxelLayer / ObstacleLayer can consume for 3D-aware costmap population.

Pipeline
--------
/depth_camera (sensor_msgs/Image, 32FC1)
    │  pinhole de-projection in camera_link frame (ROS axes: x=fwd, y=left, z=up)
    ▼
/depth_pointcloud (sensor_msgs/PointCloud2, frame_id='camera_link')
    │  Nav2 costmap applies TF camera_link → odom/map internally
    └─► Nav2 local_costmap VoxelLayer
    └─► Nav2 global_costmap ObstacleLayer

Why camera_link frame (not map)
---------------------------------
If the cloud is published in 'map' frame, the costmap resolves the
sensor origin as the position of the 'map' frame origin in the costmap
global frame — which is (0,0,0), far below the VoxelLayer z-range.
Publishing in 'camera_link' lets the costmap look up the actual camera
position via TF (drone altitude ~3 m), keeping the sensor origin inside
the voxel grid.

Assumptions
-----------
- Camera model: simple pinhole (no distortion)
- Depth encoding: 32FC1 (metres, NaN/Inf = invalid)
- Camera frame: camera_link with ROS standard axes (x=forward, y=left, z=up)
- Camera orientation: forward-facing, horizontal (identity rotation to base_link)
- The TF tree map → odom → base_link → camera_link must be live

Parameters
----------
image_width       : int   (default 640)   — depth image width  in pixels
image_height      : int   (default 480)   — depth image height in pixels
hfov_deg          : float (default 90.0)  — horizontal field of view (degrees)
downsample_factor : int   (default 8)     — stride; only every Nth pixel used
min_depth         : float (default 0.1)   — ignore readings closer than this (m)
max_depth         : float (default 8.0)   — ignore readings farther than this (m)
use_sim_time      : bool  (default True)

Subscriptions
-------------
/depth_camera  (sensor_msgs/Image)

Publications
------------
/depth_pointcloud  (sensor_msgs/PointCloud2)
"""

import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header


class DepthToCostmap(Node):

    def __init__(self):
        super().__init__('depth_to_costmap')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('hfov_deg', 90.0)
        self.declare_parameter('downsample_factor', 8)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 8.0)

        w = self.get_parameter('image_width').value
        h = self.get_parameter('image_height').value
        hfov = math.radians(self.get_parameter('hfov_deg').value)
        self._stride = self.get_parameter('downsample_factor').value
        self._min_d = self.get_parameter('min_depth').value
        self._max_d = self.get_parameter('max_depth').value

        # Pinhole intrinsics (square pixels assumed)
        self._fx = (w / 2.0) / math.tan(hfov / 2.0)
        self._fy = self._fx
        self._cx = w / 2.0
        self._cy = h / 2.0

        self.get_logger().info(
            f'Camera intrinsics: {w}x{h} px, HFOV={math.degrees(hfov):.1f}°, '
            f'fx={self._fx:.1f}, stride={self._stride}'
        )

        # ── QoS ─────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscribers / Publishers ─────────────────────────────────────────
        self._depth_sub = self.create_subscription(
            Image,
            '/depth_camera',
            self._depth_cb,
            reliable_qos,
        )
        self._cloud_pub = self.create_publisher(PointCloud2, '/depth_pointcloud', 10)

        self.get_logger().info('depth_to_costmap ready — publishing /depth_pointcloud in camera_link frame')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        # Decode 32FC1 depth image
        try:
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )
        except Exception as exc:
            self.get_logger().error(f'Depth decode failed: {exc}')
            return

        points = self._deproject(depth)
        if not points:
            return

        cloud = self._build_cloud(points, msg.header.stamp)
        self._cloud_pub.publish(cloud)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _deproject(self, depth: np.ndarray):
        """
        De-project every (stride)th pixel into camera_link ROS-standard frame.

        camera_link axes (identity rotation to base_link):
          x = forward  (depth axis for a forward-facing camera)
          y = left     (negative of image u-axis which increases rightward)
          z = up       (negative of image v-axis which increases downward)

        Returns list of [x, y, z] camera_link-frame points.
        """
        rows, cols = depth.shape
        pts = []
        for v in range(0, rows, self._stride):
            for u in range(0, cols, self._stride):
                d = depth[v, u]
                if not np.isfinite(d) or d < self._min_d or d > self._max_d:
                    continue
                x =  d                              # forward
                y = -(u - self._cx) * d / self._fx  # left
                z = -(v - self._cy) * d / self._fy  # up
                pts.append((x, y, z))
        return pts

    @staticmethod
    def _build_cloud(points: list, stamp) -> PointCloud2:
        """Pack list of (x,y,z) tuples into a sensor_msgs/PointCloud2."""
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        raw = bytearray()
        for p in points:
            raw += struct.pack('fff', float(p[0]), float(p[1]), float(p[2]))

        header = Header()
        header.stamp = stamp
        header.frame_id = 'camera_link'   # sensor frame — costmap resolves TF internally

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12          # 3 × 4 bytes
        cloud.row_step = 12 * len(points)
        cloud.data = bytes(raw)
        cloud.is_dense = True
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = DepthToCostmap()
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
