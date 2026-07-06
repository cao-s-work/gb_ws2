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
        super().__init__("d2")
        self.sub = self.create_subscription(PointCloud2, "/points_nav", self.cb, 10)
    def cb(self, m):
        global pts, done
        for i in range(min(2000, m.width)):
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
print(f"Total: {len(arr)} pts, frame: base_link")

# STOP zone: 0.3-1.0m ahead, ±0.5m wide, z 0-1m
stop_mask = (arr[:,0] > 0.3) & (arr[:,0] < 1.0) & (abs(arr[:,1]) < 0.5) & (arr[:,2] > 0.0) & (arr[:,2] < 1.0)
stop_pts = arr[stop_mask]
print(f"STOP zone (0.3-1.0m ahead): {len(stop_pts)} pts")
if len(stop_pts) > 0:
    print(f"  min x={stop_pts[:,0].min():.2f}, max x={stop_pts[:,0].max():.2f}")
    print(f"  min y={stop_pts[:,1].min():.2f}, max y={stop_pts[:,1].max():.2f}")

# Near zone: 0-3m ahead
near = arr[arr[:,0] > 0]
print(f"Points ahead (x>0): {len(near)}")
if len(near) > 0:
    closest = near[near[:,0].argmin()]
    print(f"Closest: x={closest[0]:.2f}, y={closest[1]:.2f}, z={closest[2]:.2f}")

node.destroy_node()
rclpy.shutdown()
