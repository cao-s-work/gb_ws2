#!/usr/bin/env python3
import struct, time, sys
sys.path.insert(0, "/opt/ros/humble/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

rclpy.init()
pts = []
done = False

class C(Node):
    def __init__(self):
        super().__init__("d3")
        self.sub = self.create_subscription(PointCloud2, "/cloud_registered_body", self.cb, 10)
    def cb(self, m):
        global pts, done
        for i in range(m.width):
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

arr = np.array(pts)
print(f"Total: {len(arr)} pts, frame: body")
print(f"X: min={arr[:,0].min():.2f}, max={arr[:,0].max():.2f}, front(x>0)={np.sum(arr[:,0]>0)}, back(x<0)={np.sum(arr[:,0]<0)}")
print(f"Y: min={arr[:,1].min():.2f}, max={arr[:,1].max():.2f}")
print(f"Z: min={arr[:,2].min():.2f}, max={arr[:,2].max():.2f}")

# Check ODometry frame info
node.destroy_node()
rclpy.shutdown()
