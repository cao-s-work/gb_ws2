#!/usr/bin/env python3
import math
import random
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class TestObstacleCloudNode(Node):
    def __init__(self):
        super().__init__("test_obstacle_cloud_node")

        self.declare_parameter("output_topic", "/points_nav_test")
        self.declare_parameter("mode", "none")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("num_points", 20)

        self.output_topic = self.get_parameter("output_topic").value
        self.mode = self.get_parameter("mode").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.num_points = int(self.get_parameter("num_points").value)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            f"Publishing test obstacle cloud: topic={self.output_topic}, "
            f"mode={self.mode}, frame_id={self.frame_id}, rate={self.publish_rate}Hz"
        )

    def make_points(self):
        if self.mode == "none":
            return []

        if self.mode == "stop":
            cx, cy, cz = 0.60, 0.00, 0.30
        elif self.mode == "slow":
            cx, cy, cz = 1.10, 0.00, 0.30
        else:
            self.get_logger().warn_once(f"Unknown mode={self.mode}, publishing empty cloud")
            return []

        points = []
        for _ in range(self.num_points):
            # Small deterministic-ish cluster inside the intended polygon.
            x = cx + random.uniform(-0.03, 0.03)
            y = cy + random.uniform(-0.03, 0.03)
            z = cz + random.uniform(-0.02, 0.02)
            points.append((x, y, z))
        return points

    def on_timer(self):
        """Publish a point cloud with a FRESH timestamp each frame."""
        points = self.make_points()

        msg = PointCloud2()
        # Use a timestamp slightly in the past to match TF pipeline delay.
        # collision_monitor looks up TF at the pointcloud's timestamp.
        # The TF data (odom_lio -> base_link) has ~0.4s pipeline delay from FAST-LIO,
        # so a "now" stamp appears in the future relative to cached TF.
        # Offset 0.7s keeps stamp safely behind the TF frontier.
        now = self.get_clock().now()
        past = now - rclpy.duration.Duration(seconds=0.7)
        msg.header.stamp = past.to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        if points:
            msg.data = b"".join(struct.pack("<fff", x, y, z) for x, y, z in points)
        else:
            msg.data = b""

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestObstacleCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
