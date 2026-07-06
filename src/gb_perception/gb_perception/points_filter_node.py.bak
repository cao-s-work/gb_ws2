#!/usr/bin/env python3
"""
gb_perception / points_filter_node.py

Subscribe to /cloud_body (or /cloud_registered), filter points for Nav2:
  - Convert to PointCloud2 if needed
  - Remove NaN
  - Crop by height (z_min, z_max)
  - Crop by radial range (range_min, range_max)
  - Optional voxel downsampling (voxel_leaf_size)
  - Optional self-body crop (crop_self box)
  - Remap to target_frame via TF
  - Publish /points_nav

Priority input: /cloud_body (body-frame point cloud for local obstacle detection)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import numpy as np
import struct
import math

# Try to import tf2 - it's optional (without it the node still works but can't remap frames)
try:
    import tf2_ros
    import tf2_geometry_msgs
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


def point_cloud2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Convert sensor_msgs/PointCloud2 to Nx3 numpy array (x, y, z)."""
    # Find x, y, z field offsets
    offsets = {}
    for field in msg.fields:
        if field.name in ('x', 'y', 'z'):
            offsets[field.name] = field.offset

    if len(offsets) < 3:
        return np.zeros((0, 3), dtype=np.float32)

    # Parse point data
    points_list = []
    point_step = msg.point_step
    data = msg.data

    for i in range(msg.width):
        base = i * point_step
        x = struct.unpack_from('<f', data, base + offsets['x'])[0]
        y = struct.unpack_from('<f', data, base + offsets['y'])[0]
        z = struct.unpack_from('<f', data, base + offsets['z'])[0]
        points_list.append((x, y, z))

    return np.array(points_list, dtype=np.float32)


def numpy_to_point_cloud2(points: np.ndarray, frame_id: str,
                          stamp: Time = None) -> PointCloud2:
    """Convert Nx3 numpy array back to sensor_msgs/PointCloud2."""
    n = len(points)
    if n == 0:
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = 0
        return msg

    # Build fields (XYZ only - minimal)
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    point_step = 12  # 3 * float32
    row_step = point_step * n

    # Pack data
    data = np.ascontiguousarray(points, dtype=np.float32).tobytes()

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = row_step
    msg.data = data
    msg.is_dense = True

    return msg


class PointsFilterNode(Node):
    """Filter FAST-LIO point clouds for Nav2 consumption."""

    def __init__(self):
        super().__init__('points_filter_node')

        # --- Parameters ---
        self.declare_parameter('input_topic', '/cloud_body')
        self.declare_parameter('output_topic', '/points_nav')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 1.20)
        self.declare_parameter('range_min', 0.20)
        self.declare_parameter('range_max', 6.00)
        self.declare_parameter('voxel_leaf_size', 0.05)
        self.declare_parameter('remove_nan', True)
        self.declare_parameter('crop_self', True)
        self.declare_parameter('self_box_x_min', -0.45)
        self.declare_parameter('self_box_x_max', 0.45)
        self.declare_parameter('self_box_y_min', -0.30)
        self.declare_parameter('self_box_y_max', 0.30)
        self.declare_parameter('self_box_z_min', -0.30)
        self.declare_parameter('self_box_z_max', 0.60)
        self.declare_parameter('restamp_output', True)
        self.declare_parameter('restamp_offset', 0.1)
        self.declare_parameter('tf_republish_enabled', False)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.voxel_leaf_size = self.get_parameter('voxel_leaf_size').value
        self.remove_nan = self.get_parameter('remove_nan').value
        self.crop_self = self.get_parameter('crop_self').value
        self.sx_min = self.get_parameter('self_box_x_min').value
        self.sx_max = self.get_parameter('self_box_x_max').value
        self.sy_min = self.get_parameter('self_box_y_min').value
        self.sy_max = self.get_parameter('self_box_y_max').value
        self.sz_min = self.get_parameter('self_box_z_min').value
        self.sz_max = self.get_parameter('self_box_z_max').value
        self.restamp_output = self.get_parameter('restamp_output').value
        self.restamp_offset = self.get_parameter('restamp_offset').value
        self.tf_republish_enabled = self.get_parameter('tf_republish_enabled').value

        # Stats
        self.frame_count = 0
        self.total_points_in = 0
        self.total_points_out = 0

        # QoS: depth 10, reliable (to match FAST-LIO publisher)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, qos)
        self.sub  # prevent unused warning

        # Publisher
        self.pub = self.create_publisher(
            PointCloud2, output_topic, qos)

        # TF buffer (optional - for frame transformation)
        self.tf_buffer = None
        self.tf_listener = None
        if TF2_AVAILABLE:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            # Optional: TF broadcaster for restamping the odom_lio→base_link transform
            # Only enabled when tf_republish_enabled=true (default: false)
            # This is a workaround for Livox hardware timestamp vs system clock offset.
            # When disabled, FAST-LIO is the sole authority for odom_lio→base_link TF.
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            if self.tf_republish_enabled:
                self.tf_repub_timer = self.create_timer(0.05, self.republish_tf)
                self.get_logger().info(
                    f'TF republisher active: odom_lio→base_link restamped with offset={self.restamp_offset:.1f}s')
            self._last_tf_corrected = None

        self.get_logger().info(
            f'PointsFilterNode started:\n'
            f'  input_topic:  {input_topic}\n'
            f'  output_topic: {output_topic}\n'
            f'  target_frame: {self.target_frame}\n'
            f'  z_range:      [{self.z_min:.2f}, {self.z_max:.2f}]\n'
            f'  range:        [{self.range_min:.2f}, {self.range_max:.2f}]\n'
            f'  voxel_leaf:   {self.voxel_leaf_size}\n'
            f'  crop_self:    {self.crop_self}\n'
            f'  restamp:      {self.restamp_output} (offset={self.restamp_offset:.1f}s)\n'
            f'  tf_republish: {self.tf_republish_enabled}\n'
            f'  tf_available: {TF2_AVAILABLE}'
        )

        if self.restamp_output:
            self.get_logger().info(
                'points_filter_node restamp_output enabled for /points_nav')
            if TF2_AVAILABLE and self.tf_republish_enabled:
                self.get_logger().info(
                    f'TF republisher active: odom_lio→base_link restamped with offset={self.restamp_offset:.1f}s')

        # Periodic stats
        self.stats_timer = self.create_timer(30.0, self.print_stats)

    def republish_tf(self):
        """Republish odom_lio→base_link TF with corrected (system) timestamp.
        FAST-LIO uses Livox hardware timestamps for TF; this restamps them
        so collision_monitor / local_costmap can look up transforms that
        match the restamped /points_nav timestamps.
        """
        if not TF2_AVAILABLE or self.tf_buffer is None:
            return
        try:
            # Look up the latest transform
            t = self.tf_buffer.lookup_transform(
                'odom_lio', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            # Republish with corrected timestamp
            now = self.get_clock().now()
            corrected_stamp = (now - Duration(seconds=self.restamp_offset)).to_msg()
            t.header.stamp = corrected_stamp
            self.tf_broadcaster.sendTransform(t)
            # Cache for next call
            self._last_tf_corrected = t
        except Exception:
            pass  # TF not available yet, try next tick

    def _voxel_filter(self, points: np.ndarray) -> np.ndarray:
        """Simple voxel grid downsampling."""
        if len(points) == 0:
            return points
        leaf = self.voxel_leaf_size
        if leaf <= 0:
            return points

        # Compute voxel indices
        idx = np.floor(points / leaf).astype(np.int64)
        # Use a hash to deduplicate voxels
        # For deterministic order, use lexsort + unique
        order = np.lexsort((idx[:, 2], idx[:, 1], idx[:, 0]))
        idx = idx[order]
        points = points[order]

        diff = np.any(np.diff(idx, axis=0) != 0, axis=1)
        mask = np.concatenate(([True], diff))
        return points[mask]

    def _crop_self_body(self, points: np.ndarray) -> np.ndarray:
        """Remove points that fall inside the robot's self-body box."""
        if len(points) == 0:
            return points
        mask = ~(
            (points[:, 0] >= self.sx_min) & (points[:, 0] <= self.sx_max) &
            (points[:, 1] >= self.sy_min) & (points[:, 1] <= self.sy_max) &
            (points[:, 2] >= self.sz_min) & (points[:, 2] <= self.sz_max)
        )
        return points[mask]

    def _transform_points(self, points: np.ndarray,
                          src_frame: str, dst_frame: str,
                          stamp: rclpy.time.Time) -> np.ndarray:
        """Transform points from src_frame to dst_frame using TF."""
        if not TF2_AVAILABLE or self.tf_buffer is None:
            self.get_logger().warn(
                f'TW: Cannot transform {src_frame} -> {dst_frame}, '
                f'TF2 not available.',
                throttle_duration_sec=30.0)
            return points

        try:
            t = self.tf_buffer.lookup_transform(
                dst_frame, src_frame, stamp, timeout=rclpy.duration.Duration(seconds=0.1))
            # Manual transform for Nx3 array
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            # Rotation matrix from quaternion
            xx = qx * qx
            yy = qy * qy
            zz = qz * qz
            xy = qx * qy
            xz = qx * qz
            yz = qy * qz
            wx = qw * qx
            wy = qw * qy
            wz = qw * qz

            R = np.array([
                [1 - 2*(yy+zz), 2*(xy-wz), 2*(xz+wy)],
                [2*(xy+wz), 1 - 2*(xx+zz), 2*(yz-wx)],
                [2*(xz-wy), 2*(yz+wx), 1 - 2*(xx+yy)],
            ], dtype=np.float32)

            transformed = points @ R.T + np.array([tx, ty, tz], dtype=np.float32)
            return transformed

        except Exception as e:
            self.get_logger().warn(
                f'TF transform {src_frame}->{dst_frame} failed: {e}',
                throttle_duration_sec=10.0)
            return points

    def cloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud."""
        self.frame_count += 1

        # Convert to numpy
        points = point_cloud2_to_numpy(msg)
        n_in = len(points)
        self.total_points_in += n_in

        if n_in == 0:
            return

        src_frame = msg.header.fix_frame_id() if hasattr(msg.header, 'fix_frame_id') else msg.header.frame_id

        # Step 1: Remove NaN / Inf
        if self.remove_nan:
            mask = np.isfinite(points).all(axis=1)
            points = points[mask]

        # Step 2: Height crop
        mask_z = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        points = points[mask_z]

        # Step 3: Radial range crop (from origin in the current frame)
        dist = np.linalg.norm(points, axis=1)
        mask_r = (dist >= self.range_min) & (dist <= self.range_max)
        points = points[mask_r]

        # Step 4: Self-body crop (if in body frame)
        if self.crop_self:
            points = self._crop_self_body(points)

        # Step 5: Voxel downsampling
        points = self._voxel_filter(points)

        # Step 6: TF transform to target_frame
        if src_frame != self.target_frame:
            ros_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            points = self._transform_points(points, src_frame, self.target_frame, ros_stamp)

        n_out = len(points)
        self.total_points_out += n_out

        # Publish
        if self.restamp_output:
            out_stamp = (self.get_clock().now() - Duration(seconds=self.restamp_offset)).to_msg()
        else:
            out_stamp = msg.header.stamp
        out_msg = numpy_to_point_cloud2(points, self.target_frame, out_stamp)
        self.pub.publish(out_msg)

        # Log occasional stats
        if self.frame_count % 50 == 0:
            ratio = (n_out / n_in * 100) if n_in > 0 else 0
            self.get_logger().info(
                f'[frame {self.frame_count}] {n_in} -> {n_out} pts '
                f'({ratio:.1f}%), frame={src_frame}->{self.target_frame}')

    def print_stats(self):
        """Periodic statistics output."""
        if self.frame_count == 0:
            self.get_logger().info('Stats: no frames received yet.')
            return
        avg_in = self.total_points_in / self.frame_count
        avg_out = self.total_points_out / self.frame_count
        ratio = (self.total_points_out / max(self.total_points_in, 1)) * 100
        self.get_logger().info(
            f'=== STATS ({self.frame_count} frames) ===\n'
            f'  avg in/out: {avg_in:.0f} / {avg_out:.0f} pts\n'
            f'  retention:  {ratio:.1f}%\n'
            f'  CPU:        (check externally)')


def main(args=None):
    rclpy.init(args=args)
    node = PointsFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final stats
        node.print_stats()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Context may already be shut down by launch system
            pass


if __name__ == '__main__':
    main()
