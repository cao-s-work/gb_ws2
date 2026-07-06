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
        super().__init__("d7")
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

        # Detailed STOP zone breakdown
        print(f"=== STOP 多边形细查 (y: ±0.35m, z: 0-1m) ===")
        for xlo, xhi in [(0.3, 0.5), (0.5, 0.7), (0.7, 0.9)]:
            zone = arr[(arr[:,0] > xlo) & (arr[:,0] < xhi) & (abs(arr[:,1]) < 0.35) & (arr[:,2] > 0.0) & (arr[:,2] < 1.0)]
            print(f"  {xlo}-{xhi}m 前方: {len(zone)} 点")
            if len(zone) > 0 and len(zone) <= 5:
                for p in zone:
                    print(f"    -> 前{p[0]:.2f}m {'左' if p[1]<0 else '右'}{abs(p[1]):.2f}m 高{p[2]:.2f}m")

        # Wider check: what IS within 2m at center
        center = arr[(arr[:,0] > 0.2) & (arr[:,0] < 2.0) & (abs(arr[:,1]) < 1.0)]
        print(f"\n前方 0.2-2.0m, |y|<1.0m: {len(center)} 点")

        # Closest forward point in center lane
        center_close = arr[(arr[:,0] > 0.05) & (abs(arr[:,1]) < 0.5)]
        if len(center_close) > 0:
            closest = center_close[center_close[:,0].argmin()]
            print(f"中心车道最近点: 前{closest[0]:.3f}m {'左' if closest[1]<0 else '右'}{abs(closest[1]):.3f}m")

        done = True

node = C()
t0 = time.time()
while not done and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
