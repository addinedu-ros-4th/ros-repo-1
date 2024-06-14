import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import socket
import struct
import time
import cv2
import numpy as np
from tracker_tcpip import ObjectTracker

class IPSubscriber(Node):
    def __init__(self):
        super().__init__('ip_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ip_address',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ip_address = None

    def listener_callback(self, msg):
        self.get_logger().info(f'Received IP Address: {msg.data}')
        self.ip_address = msg.data

class TCPIPClientNode(Node):
    def __init__(self, ip_address, model_path, object_name):
        super().__init__('tcpip_client_node')
        self.client_socket = None
        self.connection = None
        self.connect_to_server(ip_address)
        self.ip_address = ip_address
        self.model_path = model_path
        self.object_name = object_name

        self.receive_thread = threading.Thread(target=self.receive_video)
        self.receive_thread.start()
        
        self.object_tracker = ObjectTracker(self.model_path, self.object_name)
        self.object_tracker.start()

    def connect_to_server(self, ip_address):
        while True:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((ip_address, 5607))
                self.connection = self.client_socket.makefile('rb')
                self.get_logger().info(f"Connected to server at {ip_address}")
                break
            except Exception as e:
                self.get_logger().error(f"Failed to connect to server at {ip_address}: {e}")
                time.sleep(5)

    def receive_video(self):
        while True:
            try:
                while True:
                    packed_msg_size = self.connection.read(struct.calcsize('<L'))
                    msg_size = struct.unpack('<L', packed_msg_size)[0]
                    frame_data = self.connection.read(msg_size)
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    if frame is not None and frame.size != 0:
                        # test = frame.copy()
                        self.object_tracker.add_frame(frame)
                        # cv2.imshow('Video', frame)
                        # if cv2.waitKey(1) & 0xFF == ord('q'):
                        #     break
                    else:
                        self.get_logger().error("Invalid frame received")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                self.reconnect_to_server()

    def reconnect_to_server(self):
        self.get_logger().warning("Lost connection to server. Reconnecting...")
        self.client_socket.close()
        self.connection.close()
        self.connect_to_server(self.ip_address)

    def destroy(self):
        self.tracking_controller.destroy_node()
        self.executor.shutdown()
        self.executor_thread.join()
        self.object_tracker.running = False
        self.object_tracker.join()

def main(args=None):
    rclpy.init(args=args)

    ip_address = "192.168.0.63"

    model_path = 'best.pt'
    object_name = 'wook' # db에서 제공받은 이름

    tcpip_client_node = TCPIPClientNode(ip_address, model_path, object_name)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tcpip_client_node)

    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    tcpip_client_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()