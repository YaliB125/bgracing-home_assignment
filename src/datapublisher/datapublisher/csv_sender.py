import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pandas as pd
import time

class CsvSender(Node):
    def __init__(self):
        super().__init__('csv_sender_node')
        self.publisher_ = self.create_publisher(Point, 'cone_locations', 10)
        csv_file = '/ros2_ws/data/BrandsHatchLayout.csv'
        self.data_list = []
        self.current_index = 0
        
        try:
            self.df = pd.read_csv(csv_file).dropna()
            self.get_logger().info(f'Loaded {len(self.df)} points from {csv_file}')
            self.data_iterator = self.df.iterrows()
            self.timer = self.create_timer(0.1, self.publish_point)
            
        except FileNotFoundError:
            self.get_logger().error(f'Could not find file: {csv_file}')
        except Exception as e:
            self.get_logger().error(f'Error loading file: {str(e)}')

    def publish_point(self):
        try:
            index, row = next(self.data_iterator)
            msg = Point()
            msg.x = float(row['x'])
            msg.y = float(row['y'])
            msg.z = 0.0
            
            self.publisher_.publish(msg)
            if index % 10 == 0:
                self.get_logger().info(f'Publishing index {index}: X={msg.x:.2f}, Y={msg.y:.2f}')
                
        except StopIteration:
            self.get_logger().info('End of file reached, restarting...')
            self.data_iterator = self.df.iterrows()
        except KeyError:
            self.get_logger().error('Columns "x" or "y" not found in CSV!')

def main(args=None):
    rclpy.init(args=args)
    node = CsvSender()  # <--- הנה התיקון הקריטי
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()