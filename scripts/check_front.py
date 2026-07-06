#!/usr/bin/env python3
"""Quick check: what's in front of the dog via /points_nav"""
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import struct

rclpy.init()
node = rclpy.create_node('pc_front_check')
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

def cb(msg):
    total = msg.width
    pts = []
    for i in range(min(total, 3000)):
        off = i * msg.point_step
        x = struct.unpack_from('<f', msg.data, off)[0]
        y = struct.unpack_from('<f', msg.data, off+4)[0]
        z = struct.unpack_from('<f', msg.data, off+8)[0]
        if abs(x) < 5 and abs(y) < 5:
            pts.append((x, y, z))
    
    front = [(x,y,z) for x,y,z in pts if x > 0.20]
    stop_zone = [(x,y,z) for x,y,z in pts if 0.45 <= x <= 0.85 and abs(y) <= 0.35]
    
    print(f'Total={total}, front(>0.20m)={len(front)}, STOP_zone(0.45-0.85,|y|<0.35)={len(stop_zone)}')
    
    if len(stop_zone) > 0:
        print(f'⚠️  OBSTACLE DETECTED in STOP zone!')
        for p in sorted(stop_zone, key=lambda p: p[0])[:10]:
            print(f'  🚧 x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f}')
    else:
        print('✅ No obstacle in STOP zone (0.45-0.85m ahead)')
    
    # Show closest front points
    print('Closest front points:')
    for p in sorted(front, key=lambda p: p[0])[:10]:
        print(f'  📍 x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f}')
    
    rclpy.shutdown()

sub = node.create_subscription(PointCloud2, '/points_nav', cb, qos)
rclpy.spin_once(node, timeout_sec=5.0)
node.destroy_node()
