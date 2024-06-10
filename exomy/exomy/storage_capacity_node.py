import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import psutil

class StorageCapacityNode(Node):
    def __init__(self):
        super().__init__('storage_capacity_node')
        self.publisher_ = self.create_publisher(Float32, 'storage_capacity', 10)
        self.timer = self.create_timer(5.0, self.publish_storage_capacity)
        self.get_logger().info('Storage Capacity Node has been started.')

    def get_storage_capacity(self):
        total_capacity = psutil.disk_usage('/').total
        return total_capacity

    def publish_storage_capacity(self):
        total_capacity = self.get_storage_capacity()
        if total_capacity is not None:
            used_capacity = psutil.disk_usage('/').used
            remaining_capacity = total_capacity - used_capacity
            msg = Float32()
            msg.data = float(remaining_capacity)  # Convert to float
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published Remaining Storage Capacity: {remaining_capacity} bytes')


def main(args=None):
    rclpy.init(args=args)
    node = StorageCapacityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
