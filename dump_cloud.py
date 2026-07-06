import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import sys

class CloudDumper(Node):
    def __init__(self, topic):
        super().__init__('cloud_dumper')
        self.done = False
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        n = msg.width
        ps = msg.point_step
        data = bytes(msg.data)
        step = max(1, n // 20)
        sys.stdout.write(f"total={n}, frame={msg.header.frame_id}, step={ps}\n")
        for i in range(0, min(n, 200), step):
            base = i * ps
            x = struct.unpack_from('<f', data, base)[0]
            y = struct.unpack_from('<f', data, base + 4)[0]
            z = struct.unpack_from('<f', data, base + 8)[0]
            sys.stdout.write(f"  [{i}] x={x:.2f} y={y:.2f} z={z:.2f}\n")
        sys.stdout.flush()
        rclpy.shutdown()

rclpy.init()
node = CloudDumper(sys.argv[1])
try:
    rclpy.spin_once(node, timeout_sec=3.0)
except:
    pass
node.destroy_node()
