import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import random

class BatteryPublisher(Node):

    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(BatteryState, 'battery_state', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.battery_state = BatteryState()

    def timer_callback(self):
        self.battery_state.voltage = 12.5
        self.battery_state.current = 1.5
        self.battery_state.charge = 50.0
        self.battery_state.capacity = 100.0
        self.battery_state.design_capacity = 100.0
        self.battery_state.percentage = random.randint(1, 99) / 100.0
        self.publisher_.publish(self.battery_state)
        self.get_logger().info('Publishing: "%s"' % self.battery_state)

def main(args=None):
    rclpy.init(args=args)
    battery_publisher = BatteryPublisher()
    rclpy.spin(battery_publisher)
    battery_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
