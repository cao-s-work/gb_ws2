#!/usr/bin/env python3
"""
gb_lio / lio_odom_2d_projector.py

将 FAST-LIO 3D /Odometry (camera_init→livox_imu_link) 投影为
2D /Odometry_2d (camera_init→base_link)，供 Nav2 controller/smoother 使用。

流程:
  1. 订阅 /Odometry
  2. 通过 TF camera_init→base_link 获取当前机身 2D 位姿
  3. 投影: z=0, roll=0, pitch=0
  4. 保留平面 twist
  5. 发布 /Odometry_2d
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q: Quaternion) -> float:
    """Extract yaw from quaternion (rotation around Z)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Quaternion:
    """Build quaternion from yaw-only rotation."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class LioOdom2DProjector(Node):
    """Project FAST-LIO 3D odometry to Nav2-friendly 2D odom."""

    def __init__(self):
        super().__init__('lio_odom_2d_projector')

        # Parameters
        self.declare_parameter('input_odom_topic', '/Odometry')
        self.declare_parameter('output_odom_topic', '/Odometry_2d')
        self.declare_parameter('odom_frame', 'camera_init')
        self.declare_parameter('base_frame', 'base_link')

        input_topic = self.get_parameter('input_odom_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, input_topic, self.odom_callback, qos)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, output_topic, qos)

        self.get_logger().info(
            f'LioOdom2DProjector started:\n'
            f'  input:  {input_topic}\n'
            f'  output: {output_topic}\n'
            f'  frames: {self.odom_frame} → {self.base_frame}')

    def odom_callback(self, msg: Odometry):
        """Receive FAST-LIO /Odometry, publish 2D-projected version."""
        # Look up TF: camera_init → base_link to get actual base_link pose
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame,
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            self.get_logger().warn(
                f'TF {self.odom_frame}→{self.base_frame} lookup failed, skip',
                throttle_duration_sec=10.0)
            return

        # Extract translation + yaw
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)

        # Build 2D odom message
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame

        # Pose: 2D only (z=0, roll=0, pitch=0)
        out.pose.pose.position.x = tx
        out.pose.pose.position.y = ty
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation = yaw_to_quat(yaw)

        # Copy pose covariance from input (3x3 for x,y,yaw is fine)
        out.pose.covariance = list(msg.pose.covariance)

        # Twist: keep planar velocities, zero others
        out.twist.twist.linear.x = msg.twist.twist.linear.x
        out.twist.twist.linear.y = msg.twist.twist.linear.y
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = msg.twist.twist.angular.z
        out.twist.covariance = list(msg.twist.covariance)

        self.odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdom2DProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
