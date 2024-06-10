import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import subprocess

class CpuTemperatureNode(Node):
    def __init__(self):
        super().__init__('cpu_temperature_node')
        self.publisher_ = self.create_publisher(Float32, 'cpu_temp', 10)
        self.timer = self.create_timer(2.0, self.publish_temperature)
        self.get_logger().info('Cpu Temperature Node has been started.')

    def get_cpu_temperature(self):
        result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
        temp_str = result.stdout
        temp_value = float(temp_str.split('=')[1].split("'")[0])
        return temp_value

    def publish_temperature(self):
        temperature = self.get_cpu_temperature()
        msg = Float32()
        msg.data = temperature
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published CPU Temperature: {temperature:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = CpuTemperatureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
