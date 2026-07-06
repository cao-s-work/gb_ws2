#!/usr/bin/env python3
import struct, time, sys
sys.path.insert(0, "/opt/ros/humble/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

rclpy.init()
pts = []
done = False

class C(Node):
    def __init__(self):
        super().__init__("d")
        self.sub = self.create_subscription(PointCloud2, "/cloud_registered_body", self.cb, 10)
    def cb(self, m):
        global pts, done
        for i in range(min(10, m.width)):
            b = i * m.point_step
            x = struct.unpack_from("<f", m.data, b)[0]
            y = struct.unpack_from("<f", m.data, b+4)[0]
            z = struct.unpack_from("<f", m.data, b+8)[0]
            pts.append((x,y,z))
        done = True

node = C()
t0 = time.time()
while not done and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)
for p in pts:
    print(f"{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}")
node.destroy_node()
rclpy.shutdown()
