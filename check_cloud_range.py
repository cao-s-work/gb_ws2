import rclpy, struct, math, time, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

class CloudCheck(Node):
    def __init__(self):
        super().__init__("cloud_check2")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.results = {}
        for t in ["/cloud_registered_body", "/cloud_registered", "/points_nav"]:
            self.create_subscription(PointCloud2, t, lambda m, topic=t: self.cb(topic, m), qos)

    def cb(self, topic, msg):
        if topic in self.results: return
        pts = []
        data = bytes(msg.data)
        ps = msg.point_step
        n = min(msg.width, 2000)
        for i in range(n):
            base = i * ps
            if base + 12 > len(data): break
            x = struct.unpack_from('<f', data, base)[0]
            y = struct.unpack_from('<f', data, base+4)[0]
            z = struct.unpack_from('<f', data, base+8)[0]
            if math.isfinite(x):
                pts.append((x,y,z))
        self.results[topic] = (msg.header.frame_id, msg.width, pts)

rclpy.init()
node = CloudCheck()
t0 = time.time()
while time.time() - t0 < 8.0 and len(node.results) < 3:
    rclpy.spin_once(node, timeout_sec=0.5)

for t in ["/cloud_registered_body", "/cloud_registered", "/points_nav"]:
    if t in node.results:
        fid, w, pts = node.results[t]
        sys.stdout.write(f"\n{t} frame={fid} total={w} sampled={len(pts)}\n")
        if pts:
            xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; zs=[p[2] for p in pts]
            sys.stdout.write(f"  x: {min(xs):.1f} ~ {max(xs):.1f}\n")
            sys.stdout.write(f"  y: {min(ys):.1f} ~ {max(ys):.1f}\n")
            sys.stdout.write(f"  z: {min(zs):.1f} ~ {max(zs):.1f}\n")
    else:
        sys.stdout.write(f"\n{t} NO DATA\n")
sys.stdout.flush()
node.destroy_node()
rclpy.shutdown()
