#!/usr/bin/env python3.10
import rclpy
from nav_msgs.msg import OccupancyGrid
import yaml, numpy as np, os, time
from PIL import Image

rclpy.init()
node = rclpy.create_node('save_map')

map_data = None
def cb(msg):
    global map_data
    map_data = msg
    print(f"  got map {msg.info.width}x{msg.info.height}")

# Use default (system default) QoS from rclpy
sub = node.create_subscription(OccupancyGrid, '/map', cb, 10)

print("Waiting for /map...")
for i in range(50):
    rclpy.spin_once(node, timeout_sec=0.2)
    if map_data is not None:
        break

if map_data is None:
    print("FAIL: timeout")
    exit(1)

m = map_data
data = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
free = int(np.sum(data == 0))
occ = int(np.sum(data == 100))
unk = int(np.sum(data == -1))
t = max(free+occ+unk, 1)
print(f"Map: {m.info.width}x{m.info.height} res={m.info.resolution:.3f} origin=[{m.info.origin.position.x:.3f},{m.info.origin.position.y:.3f}]")
print(f"Free={free}({100*free/t:.1f}%) Occ={occ}({100*occ/t:.1f}%) Unk={unk}({100*unk/t:.1f}%)")

d = "/home/nvidia/gb_maps/20260703_gb_map"
os.makedirs(d, exist_ok=True)

img = np.zeros_like(data, dtype=np.uint8)
img[data==-1]=205; img[data==0]=254; img[data==100]=0
Image.fromarray(img[::-1,:]).save(os.path.join(d,"map.pgm"))

yc = {"image":"map.pgm","mode":"trinary","resolution":float(m.info.resolution),
      "origin":[float(m.info.origin.position.x),float(m.info.origin.position.y),0.0],
      "negate":0,"occupied_thresh":0.65,"free_thresh":0.25}
with open(os.path.join(d,"map.yaml"),"w") as f:
    yaml.dump(yc,f,default_flow_style=False)
print(f"\nSaved: {d}/")
node.destroy_node()
rclpy.shutdown()
