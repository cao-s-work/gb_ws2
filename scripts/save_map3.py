#!/usr/bin/env python3.10
"""Explicit TRANSIENT_LOCAL QoS subscriber to grab /map"""
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import yaml, numpy as np, os
from PIL import Image

rclpy.init()
node = rclpy.create_node('map_grabber')

qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

got = None
def cb(msg):
    global got
    got = msg
    print(f"  RECEIVED: {msg.info.width}x{msg.info.height}")

sub = node.create_subscription(OccupancyGrid, '/map', cb, qos)

print("Spinning for /map (TRANSIENT_LOCAL)...")
import time
start = time.time()
while got is None and time.time() - start < 10.0:
    rclpy.spin_once(node, timeout_sec=0.1)

if got is None:
    print("FAIL: still no map after 10s")
    node.destroy_node()
    rclpy.shutdown()
    exit(1)

m = got
data = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
free = int(np.sum(data == 0))
occ = int(np.sum(data == 100))
unk = int(np.sum(data == -1))
total = max(free+occ+unk, 1)
print(f"\nMap: {m.info.width}x{m.info.height} res={m.info.resolution:.3f}")
print(f"Origin: [{m.info.origin.position.x:.3f}, {m.info.origin.position.y:.3f}]")
print(f"Free={free}({100*free/total:.1f}%) Occ={occ}({100*occ/total:.1f}%) Unk={unk}({100*unk/total:.1f}%)")

d = "/home/nvidia/gb_maps/20260703_gb_map"
os.makedirs(d, exist_ok=True)

img = np.zeros_like(data, dtype=np.uint8)
img[data==-1]=205; img[data==0]=254; img[data==100]=0
Image.fromarray(img[::-1,:]).save(os.path.join(d, "map.pgm"))

yaml_data = {
    "image":"map.pgm","mode":"trinary",
    "resolution":float(m.info.resolution),
    "origin":[float(m.info.origin.position.x), float(m.info.origin.position.y), 0.0],
    "negate":0,"occupied_thresh":0.65,"free_thresh":0.25,
}
with open(os.path.join(d, "map.yaml"), "w") as f:
    yaml.dump(yaml_data, f, default_flow_style=False)

print(f"\nSaved: {d}/")
print(f"  map.yaml + map.pgm ({img.shape[1]}x{img.shape[0]})")
node.destroy_node()
rclpy.shutdown()
