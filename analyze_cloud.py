import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import sys

class CloudAnalyzer(Node):
    def __init__(self, topic, label):
        super().__init__('cloud_analyzer')
        self.label = label
        self.done = False
        self.sub = self.create_subscription(
            PointCloud2, topic, self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        
        n = msg.width
        ps = msg.point_step
        data = bytes(msg.data)
        
        in_front = 0
        near_body = 0
        far_away = 0
        for i in range(min(n, 2000)):
            base = i * ps
            if base + 12 > len(data):
                break
            x = struct.unpack_from('<f', data, base)[0]
            y = struct.unpack_from('<f', data, base + 4)[0]
            z = struct.unpack_from('<f', data, base + 8)[0]
            dist = (x*x + y*y)**0.5
            if 0.3 < x < 1.5 and abs(y) < 0.5 and 0.05 < z < 1.2:
                in_front += 1
            if abs(x) < 0.5 and abs(y) < 0.35:
                near_body += 1
            if dist > 3.0:
                far_away += 1
        
        sys.stdout.write(f"[{self.label}] total={n} in_front={in_front} near_body={near_body} far={far_away}\n")
        sys.stdout.flush()
        rclpy.shutdown()

def main():
    rclpy.init()
    args = sys.argv[1:]
    if len(args) < 2:
        print("Usage: python analyze_cloud.py <topic> <label>")
        sys.exit(1)
    
    node = CloudAnalyzer(args[0], args[1])
    try:
        rclpy.spin_once(node, timeout_sec=3.0)
    except:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
