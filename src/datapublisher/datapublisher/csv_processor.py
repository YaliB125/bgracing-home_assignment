import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

class CsvProcessor(Node):
    def __init__(self):
        super().__init__('csv_processor_node')
        self.subscription = self.create_subscription(
            Point,
            'cone_locations',
            self.listener_callback,
            10)
        self.x_data = []
        self.y_data = []
        self.get_logger().info('Waiting for data to calculate Middle Path...')

    def listener_callback(self, msg):
        self.x_data.append(msg.x)
        self.y_data.append(msg.y)
        
        count = len(self.x_data)
        if count % 50 == 0:
            self.get_logger().info(f'Collected {count} points...')
   
        if count >= 312:
            self.calculate_and_save_middle_path()
            self.x_data = [] 
            self.y_data = []

    def calculate_and_save_middle_path(self):
     
        half = len(self.x_data) // 2
        
        left_x = self.x_data[:half]
        left_y = self.y_data[:half]
        
        right_x = self.x_data[half:]
        right_y = self.y_data[half:]
        mid_x = []
        mid_y = []
        
      
        min_len = min(len(left_x), len(right_x))
        
        for i in range(min_len):
            avg_x = (left_x[i] + right_x[i]) / 2.0
            avg_y = (left_y[i] + right_y[i]) / 2.0
            mid_x.append(avg_x)
            mid_y.append(avg_y)

        mid_x.append(mid_x[0])
        mid_y.append(mid_y[0])
        plt.figure(figsize=(10, 8))
        
        plt.plot(self.x_data, self.y_data, 'gray', alpha=0.3, label='Track Limits')
        
        plt.plot(mid_x, mid_y, 'b-', linewidth=3, label='Calculated Middle Path')
        plt.scatter(mid_x, mid_y, c='red', s=20, zorder=5)
        
        plt.title('Final Project: Middle Path Calculation')
        plt.xlabel('X [meters]')
        plt.ylabel('Y [meters]')
        plt.legend()
        plt.grid(True)
        
        output_path = '/ros2_ws/data/result.png'
        plt.savefig(output_path)
        self.get_logger().info(f'SUCCESS! Middle Path Saved to: {output_path}')

def main(args=None):
    rclpy.init(args=args)
    node = CsvProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()