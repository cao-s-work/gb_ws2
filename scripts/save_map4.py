#!/usr/bin/env python3.10
"""Grab /map from slam_toolbox with TRANSIENT_LOCAL QoS"""
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import yaml, numpy as np, os, time
from PIL import Image

rclpy.init()
node = rclpy.create_node('map_grabber')

# Try qos_profile_sensor_data first (default ROS 2 sensor QoS, should match)
got = [None]

def cb(msg):
    got[0] = msg

# Try multiple QoS profiles
qos_options = [
    ("SENSOR_DATA", qos_profile_sensor_data),
    ("TRANSIENT_LOCAL", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)),
    ("VOLATILE_RELIABLE", QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)),
    ("BEST_EFFORT", QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)),
]

for qos_name, qos in qos_options:
    sub = node.create_subscription(OccupancyGrid, '/map', cb, qos)
    print(f"Trying QoS: {qos_name}...")
    start = time.time()
    while got[0] is None and time.time() - start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if got[0] is not None:
        print(f"  SUCCESS with {qos_name}!")
        break
    print(f"  no data")

if got[0] is None:
    print("FAIL: all QoS profiles failed")
    node.destroy_node()
    rclpy.shutdown()
    exit(1)

m = got[0]
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
node.destroy_node()
rclpy.shutdown()
