#!/usr/bin/env python3
"""
gb_lio / gb_lio / icp_localization.py

PCL ICP 3D 定位节点 (纯 numpy/scipy, 无需 open3d):
  加载 FAST-LIO PCD → 订阅 /cloud_registered_body →
  ICP scan-to-map 匹配 → 发布 map→camera_init TF

Usage:
    ros2 run gb_lio icp_localization --ros-args \
        -p pcd_map:=/home/nvidia/gb_maps/fastlio_map/test.pcd \
        -p initial_pose_x:=0.0 -p initial_pose_y:=0.0 -p initial_pose_yaw:=0.0

试验性 — 精度取决于地图质量和初始猜测.
"""

import os
import time
import struct
import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ============================================================
# PCD 文件读取 (binary format)
# ============================================================
def read_pcd_binary(filepath):
    """读取 binary PCD 文件，返回 (N,3) numpy array"""
    with open(filepath, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('utf-8').strip()
            header.append(line)
            if line.startswith('DATA'):
                break

        # 解析 header 获取点数
        n_points = 0
        for line in header:
            if line.startswith('POINTS'):
                n_points = int(line.split()[1])
            if line.startswith('FIELDS'):
                fields = line.split()[1:]

        # 确定是否 binary
        is_binary = 'binary' in header[-1]

        if not is_binary:
            raise ValueError('Only binary PCD supported')

        # 读取点数据
        x_idx = fields.index('x') if 'x' in fields else 0
        y_idx = fields.index('y') if 'y' in fields else 1
        z_idx = fields.index('z') if 'z' in fields else 2

        points = np.zeros((n_points, 3), dtype=np.float32)
        point_size = len(fields) * 4  # 4 bytes per float

        for i in range(n_points):
            raw = f.read(point_size)
            if len(raw) < point_size:
                break
            x = struct.unpack_from('f', raw, x_idx * 4)[0]
            y = struct.unpack_from('f', raw, y_idx * 4)[0]
            z = struct.unpack_from('f', raw, z_idx * 4)[0]
            points[i] = [x, y, z]

        return points


def ros_cloud_to_numpy(cloud_msg):
    """ROS2 PointCloud2 → numpy (N,3)"""
    points = []
    point_step = cloud_msg.point_step
    data = cloud_msg.data
    num_points = cloud_msg.width * (cloud_msg.height or 1)

    for i in range(num_points):
        offset = i * point_step
        try:
            x, y, z = struct.unpack_from('fff', data, offset)
            points.append([x, y, z])
        except struct.error:
            break

    return np.array(points, dtype=np.float32)


def icp_point_to_point(source, target, init_pose, max_iter=50, max_dist=2.0, tol=1e-6):
    """
    简化的 point-to-point ICP.
    source: (N,3) 源点云 (当前扫描, camera_init frame)
    target: (M,3) 目标点云 (地图, map frame)
    init_pose: (4,4) 初始变换矩阵
    返回: (4,4) 变换矩阵, fitness, rmse
    """
    T = init_pose.copy()
    kdtree = KDTree(target)

    for it in range(max_iter):
        # 变换源点云
        src_transformed = (T[:3, :3] @ source.T).T + T[:3, 3]

        # 最近邻匹配
        dists, idxs = kdtree.query(src_transformed, distance_upper_bound=max_dist)
        valid = np.isfinite(dists)
        if valid.sum() < 10:
            break

        src_valid = src_transformed[valid]
        tgt_valid = target[idxs[valid]]

        # 计算质心
        src_centroid = src_valid.mean(axis=0)
        tgt_centroid = tgt_valid.mean(axis=0)

        # 去质心
        src_demean = src_valid - src_centroid
        tgt_demean = tgt_valid - tgt_centroid

        # SVD 求旋转
        H = src_demean.T @ tgt_demean
        U, S, Vt = np.linalg.svd(H)
        R_new = Vt.T @ U.T
        if np.linalg.det(R_new) < 0:
            Vt[-1, :] *= -1
            R_new = Vt.T @ U.T
        t_new = tgt_centroid - R_new @ src_centroid

        # 更新变换
        T_delta = np.eye(4)
        T_delta[:3, :3] = R_new
        T_delta[:3, 3] = t_new
        T = T_delta @ T

        # 收敛检查
        if it > 0 and np.linalg.norm(t_new) < tol:
            break

    # 最终评估
    src_final = (T[:3, :3] @ source.T).T + T[:3, 3]
    dists_final, _ = kdtree.query(src_final, distance_upper_bound=max_dist)
    valid_final = np.isfinite(dists_final)
    fitness = valid_final.sum() / len(source) if len(source) > 0 else 0
    rmse = np.sqrt(np.mean(dists_final[valid_final] ** 2)) if valid_final.sum() > 0 else 999

    return T, fitness, rmse


# ============================================================
# ROS 节点
# ============================================================
class IcpLocalization(Node):
    def __init__(self):
        super().__init__('icp_localization')

        # 参数
        self.declare_parameter('pcd_map', '')
        self.declare_parameter('voxel_size', 0.3)
        self.declare_parameter('icp_max_distance', 2.0)
        self.declare_parameter('icp_max_iterations', 50)
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'camera_init')
        self.declare_parameter('min_scan_points', 100)

        pcd_map_path = self.get_parameter('pcd_map').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.icp_max_dist = self.get_parameter('icp_max_distance').value
        self.icp_max_iter = self.get_parameter('icp_max_iterations').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.min_scan_pts = self.get_parameter('min_scan_points').value

        # 加载 PCD 地图
        self.map_points = None
        if pcd_map_path and os.path.exists(pcd_map_path):
            self.load_map(pcd_map_path)
        else:
            self.get_logger().warn(
                f'PCD map not found: "{pcd_map_path}". '
                'Run FAST-LIO mapping and call /map_save service first.'
            )

        # 初始位姿
        init_x = self.get_parameter('initial_pose_x').value
        init_y = self.get_parameter('initial_pose_y').value
        init_yaw = self.get_parameter('initial_pose_yaw').value
        c, s = np.cos(init_yaw), np.sin(init_yaw)
        self.current_pose = np.eye(4)
        self.current_pose[:3, 3] = [init_x, init_y, 0.0]
        self.current_pose[:3, :3] = np.array([
            [c, -s, 0], [s, c, 0], [0, 0, 1]
        ])

        self.fastlio_odom = None
        self.scan_count = 0

        # QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(
            PointCloud2, '/cloud_registered_body',
            self.cloud_callback, qos)
        self.create_subscription(
            Odometry, '/Odometry',
            self.odom_callback, 10)

        self.localized_odom_pub = self.create_publisher(
            Odometry, '/Odometry_localized', 10)

        self.tf_br = TransformBroadcaster(self)
        self.get_logger().info(
            f'ICP Localization ready. map={pcd_map_path}, voxel={self.voxel_size}m'
        )

    def load_map(self, path):
        self.get_logger().info(f'Loading PCD: {path}')
        try:
            points = read_pcd_binary(path)
            self.get_logger().info(f'Raw map: {len(points)} pts')

            # 降采样
            if self.voxel_size > 0 and len(points) > 1000:
                vox = self.voxel_size
                indices = np.floor(points / vox).astype(np.int32)
                _, unique_idx = np.unique(indices, axis=0, return_index=True)
                points = points[unique_idx]
                self.get_logger().info(f'Downsampled: {len(points)} pts')
            self.map_points = points

        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')

    def odom_callback(self, msg):
        self.fastlio_odom = msg

    def cloud_callback(self, msg):
        if self.map_points is None:
            return

        t_start = time.time()
        scan = ros_cloud_to_numpy(msg)
        if len(scan) < self.min_scan_pts:
            return

        # 降采样
        if self.voxel_size > 0:
            vox = self.voxel_size * 0.5
            indices = np.floor(scan / vox).astype(np.int32)
            _, unique_idx = np.unique(indices, axis=0, return_index=True)
            scan = scan[unique_idx]

        # ICP 初始猜测：优先用 FAST-LIO odom，否则用上次结果
        if self.fastlio_odom is not None:
            p = self.fastlio_odom.pose.pose.position
            q = self.fastlio_odom.pose.pose.orientation
            rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            init_guess = np.eye(4)
            init_guess[:3, :3] = rot
            init_guess[:3, 3] = [p.x, p.y, p.z]
        else:
            init_guess = self.current_pose

        # ICP 配准
        try:
            T, fitness, rmse = icp_point_to_point(
                scan, self.map_points, init_guess,
                max_iter=self.icp_max_iter,
                max_dist=self.icp_max_dist
            )
        except Exception as e:
            self.get_logger().error(f'ICP error: {e}', throttle_duration_sec=2.0)
            return

        self.current_pose = T
        self.scan_count += 1

        # 发布 TF
        t = T[:3, 3]
        quat = R.from_matrix(T[:3, :3]).as_quat()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_br.sendTransform(tf_msg)

        # 发布里程计
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.odom_frame
        odom_msg.pose.pose.position.x = float(t[0])
        odom_msg.pose.pose.position.y = float(t[1])
        odom_msg.pose.pose.position.z = float(t[2])
        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])
        self.localized_odom_pub.publish(odom_msg)

        dt = (time.time() - t_start) * 1000
        self.get_logger().info(
            f'[{self.scan_count}] ICP: fit={fitness:.3f} rmse={rmse:.3f}m '
            f'pos=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f}) {dt:.0f}ms',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = IcpLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
