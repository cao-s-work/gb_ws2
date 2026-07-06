#!/usr/bin/env python3
import struct, time, sys
sys.path.insert(0, "/opt/ros/humble/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

rclpy.init()
all_pts = []
got_body = False
got_nav = False

class C(Node):
    def __init__(self):
        super().__init__("d4")
        self.sub1 = self.create_subscription(PointCloud2, "/cloud_registered_body", self.cb1, 10)
        self.sub2 = self.create_subscription(PointCloud2, "/points_nav", self.cb2, 10)
    def cb1(self, m):
        global got_body, all_pts
        if got_body: return
        pts = []
        for i in range(m.width):
            b = i * m.point_step
            x = struct.unpack_from("<f", m.data, b)[0]
            y = struct.unpack_from("<f", m.data, b+4)[0]
            pts.append(x)
        arr = np.array(pts)
        print(f"[body] total={len(arr)}, x>0={np.sum(arr>0)}, x<0={np.sum(arr<0)}, min={arr.min():.1f}, max={arr.max():.1f}")
        got_body = True
    def cb2(self, m):
        global got_nav, all_pts
        if got_nav: return
        print(f"[nav] point_step={m.point_step}, row_step={m.row_step}, width={m.width}, height={m.height}")
        print(f"[nav] fields: {[(f.name, f.offset) for f in m.fields]}")
        pts = []
        for i in range(m.width):
            b = i * m.point_step
            x = struct.unpack_from("<f", m.data, b)[0]
            pts.append(x)
        arr = np.array(pts)
        print(f"[nav] total={len(arr)}, x>0={np.sum(arr>0)}, x<0={np.sum(arr<0)}, min={arr.min():.1f}, max={arr.max():.1f}")
        # sample first 5
        for i in range(min(5, m.width)):
            b = i * m.point_step
            x = struct.unpack_from("<f", m.data, b)[0]
            y = struct.unpack_from("<f", m.data, b+4)[0]
            z = struct.unpack_from("<f", m.data, b+8)[0]
            print(f"  pt[{i}]: ({x:.2f}, {y:.2f}, {z:.2f})")
        got_nav = True

node = C()
t0 = time.time()
while (not got_body or not got_nav) and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
