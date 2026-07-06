#!/usr/bin/env python3
"""Save /map from slam_toolbox (handles TRANSIENT_LOCAL QoS)"""
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np
from PIL import Image
import time
import os

rclpy.init()
node = rclpy.create_node('map_saver_py')

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

map_data = None
def map_cb(msg):
    global map_data
    map_data = msg

sub = node.create_subscription(OccupancyGrid, '/map', map_cb, qos)

timeout = 10.0
start = time.time()
while map_data is None and time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.1)

if map_data is None:
    print("FAIL: no map received after 10s")
    node.destroy_node()
    rclpy.shutdown()
    exit(1)

m = map_data
print(f"Map: {m.info.width}x{m.info.height}, res={m.info.resolution:.3f}")
print(f"Origin: [{m.info.origin.position.x:.3f}, {m.info.origin.position.y:.3f}]")

data = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
free = int(np.sum(data == 0))
occupied = int(np.sum(data == 100))
unknown = int(np.sum(data == -1))
total = max(free + occupied + unknown, 1)
print(f"Free: {free} ({100*free/total:.1f}%), Occupied: {occupied} ({100*occupied/total:.1f}%), Unknown: {unknown} ({100*unknown/total:.1f}%)")

map_dir = "/home/nvidia/gb_maps/20260703_gb_map"
os.makedirs(map_dir, exist_ok=True)

img_data = np.zeros_like(data, dtype=np.uint8)
img_data[data == -1] = 205
img_data[data == 0] = 254
img_data[data == 100] = 0
img = Image.fromarray(img_data[::-1, :])
img.save(os.path.join(map_dir, "map.pgm"))

yaml_data = {
    "image": "map.pgm",
    "mode": "trinary",
    "resolution": float(m.info.resolution),
    "origin": [float(m.info.origin.position.x), float(m.info.origin.position.y), 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.25,
}
with open(os.path.join(map_dir, "map.yaml"), "w") as f:
    yaml.dump(yaml_data, f, default_flow_style=False)

print(f"\nSaved: {map_dir}/")
print(f"  map.yaml")
print(f"  map.pgm ({img_data.shape[1]}x{img_data.shape[0]})")
node.destroy_node()
rclpy.shutdown()
