#!/usr/bin/env python3
import struct, time, sys
sys.path.insert(0, "/opt/ros/humble/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

rclpy.init()
done = False

class C(Node):
    def __init__(self):
        super().__init__("d8")
        self.sub = self.create_subscription(PointCloud2, "/points_nav", self.cb, 10)
    def cb(self, m):
        global done
        if done: return
        pts = []
        for i in range(m.width):
            b = i * m.point_step
            x = struct.unpack_from("<f", m.data, b)[0]
            y = struct.unpack_from("<f", m.data, b+4)[0]
            z = struct.unpack_from("<f", m.data, b+8)[0]
            pts.append((x,y,z))
        arr = np.array(pts)

        # Any points within 1m, any y
        within_1m = arr[(arr[:,0] > 0.1) & (arr[:,0] < 1.0) & (arr[:,2] > 0.01)]
        print(f"前方 0.1-1.0m, z>0.01: {len(within_1m)} 点")
        if len(within_1m) > 0:
            print(f"  x范围: {within_1m[:,0].min():.2f} - {within_1m[:,0].max():.2f}")
            print(f"  y范围: {within_1m[:,1].min():.2f} - {within_1m[:,1].max():.2f}")
            # Sample a few
            for i in range(min(5, len(within_1m))):
                p = within_1m[i]
                print(f"  [{i}] 前{p[0]:.2f}m {'左' if p[1]<0 else '右'}{abs(p[1]):.2f}m 高{p[2]:.2f}m")
        else:
            print("  完全没有！箱子没被看到")

        done = True

node = C()
t0 = time.time()
while not done and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
