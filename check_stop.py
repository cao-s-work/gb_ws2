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
        super().__init__("d5")
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

        # STOP zone: 0.3-1.0m ahead, ±0.5m wide, 0-1m height
        stop = arr[(arr[:,0] > 0.3) & (arr[:,0] < 1.0) & (abs(arr[:,1]) < 0.5) & (arr[:,2] > 0.0) & (arr[:,2] < 1.0)]
        front = arr[arr[:,0] > 0]

        print(f"Total: {len(arr)} pts")
        print(f"Front (x>0): {len(front)} pts, range x=[{front[:,0].min():.2f}, {front[:,0].max():.2f}]")
        print(f"STOP zone (0.3-1.0m): {len(stop)} pts")

        if len(front) > 0:
            # closest point in front
            closest = front[front[:,0].argmin()]
            print(f"Closest front: x={closest[0]:.3f} y={closest[1]:.3f} z={closest[2]:.3f}")

        done = True

node = C()
t0 = time.time()
while not done and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
