import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

class ScanCheck(Node):
    def __init__(self):
        super().__init__("scan_check")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(LaserScan, "/scan_local", self.cb, qos)
    def cb(self, msg):
        ranges = [r for r in msg.ranges if r > 0 and r < float('inf')]
        print(f"frame: {msg.header.frame_id}")
        print(f"total: {len(msg.ranges)} ranges, valid: {len(ranges)}")
        if ranges:
            print(f"min: {min(ranges):.2f}m, max: {max(ranges):.2f}m")
            # front sector (angles near 0)
            front = [r for i,r in enumerate(msg.ranges) if abs(msg.angle_min + i*msg.angle_increment) < 0.5 and r > 0]
            if front:
                print(f"front_0.5m: min={min(front):.2f} max={max(front):.2f}")
        rclpy.shutdown()

rclpy.init()
node = ScanCheck()
rclpy.spin(node)
