import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import os
import math
class CsvProcessor(Node):
    def __init__(self):
        super().__init__('csv_processor_node')

        self.sub = self.create_subscription(Point, 'cone_locations', self.callback, 10)

        self.left = []
        self.right = []
        self.expected_points = 312
        
        self.get_logger().info("Ready. Waiting for points...")

    def callback(self, msg):
        if msg.z == 1.0:
            self.left.append((msg.x, msg.y))
        elif msg.z == 2.0:
            self.right.append((msg.x, msg.y))
        total = len(self.left) + len(self.right)
        if total >= self.expected_points:
            self.process_smart()

    def process_smart(self):
        mid_x = []
        mid_y = []

        for lx, ly in self.left:
            
            closest_rx = 0
            closest_ry = 0
            min_dist = 999999.0 
            
            
            for rx, ry in self.right:
                dist = math.hypot(lx - rx, ly - ry)
                if dist < min_dist:
                    min_dist = dist
                    closest_rx = rx
                    closest_ry = ry
            
            mid_x.append((lx + closest_rx) / 2)
            mid_y.append((ly + closest_ry) / 2)

        plt.figure(figsize=(12,12))
        plt.plot([p[0] for p in self.left], [p[1] for p in self.left], 'g.', label='Left')
        plt.plot([p[0] for p in self.right], [p[1] for p in self.right], 'r.', label='Right')
        plt.plot(mid_x, mid_y, 'b-', linewidth=3, label='Middle')

        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.title("Correct Middle Path")
        os.makedirs("/ros2_ws/data", exist_ok=True)
        plt.savefig("/ros2_ws/data/result.png")
        plt.close()

        self.get_logger().info("SUCCESS! Saved to /ros2_ws/data/result.png")
        self.left = []
        self.right = []

def main(args=None):
    rclpy.init(args=args)
    node = CsvProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()