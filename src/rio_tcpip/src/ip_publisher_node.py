import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import netifaces

def get_ip_address(interface='wlan0'):
    addresses = netifaces.ifaddresses(interface)
    ip_info = addresses.get(netifaces.AF_INET)
    if ip_info:
        ip_address = ip_info[0].get('addr')
        return ip_address
    return None

class IPPublisher(Node):
    def __init__(self):
        super().__init__('ip_publisher')
        self.publisher_ = self.create_publisher(String, 'ip_address', 10)
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        ip_address = get_ip_address()
        if ip_address:
            msg = String()
            msg.data = ip_address
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing IP Address: {ip_address}')
        else:
            self.get_logger().warn('No IP address found for wlan0')

def main(args=None):
    rclpy.init(args=args)
    node = IPPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

