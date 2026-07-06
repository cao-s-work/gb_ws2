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
        super().__init__("d6")
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

        # STOP zone: 0.45-0.85m ahead, ±0.35m wide, 0-1m high
        stop = arr[(arr[:,0] > 0.45) & (arr[:,0] < 0.85) & (abs(arr[:,1]) < 0.35) & (arr[:,2] > 0.0) & (arr[:,2] < 1.0)]
        # All front points
        front = arr[arr[:,0] > 0.1]
        # Near zone (0.1-2.0m)
        near = arr[(arr[:,0] > 0.1) & (arr[:,0] < 2.0)]

        print(f"=== 障碍点分布 (base_link 坐标系) ===")
        print(f"前方 x>0.1: {len(front)} 个点")
        print(f"近区 0.1-2.0m: {len(near)} 个点")
        print(f"STOP 区 (0.45-0.85m, |y|<0.35): {len(stop)} 个点")
        
        if len(stop) > 0:
            print(f"\nSTOP 区内障碍点详情:")
            for i, p in enumerate(stop):
                print(f"  [{i}] 前方 {p[0]:.2f}m,  {'左' if p[1] < 0 else '右'}侧 {abs(p[1]):.2f}m,  高度 {p[2]:.2f}m")

        if len(near) > 0:
            # Group by 0.5m bins
            print(f"\n按距离分布:")
            for dmin in [0.1, 0.5, 1.0, 1.5]:
                dmax = dmin + 0.5
                cnt = np.sum((near[:,0] >= dmin) & (near[:,0] < dmax))
                left = np.sum((near[:,0] >= dmin) & (near[:,0] < dmax) & (near[:,1] < 0))
                right = np.sum((near[:,0] >= dmin) & (near[:,0] < dmax) & (near[:,1] >= 0))
                print(f"  {dmin:.1f}-{dmax:.1f}m: {cnt} 点 (左{left} 右{right})")

        done = True

node = C()
t0 = time.time()
while not done and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
