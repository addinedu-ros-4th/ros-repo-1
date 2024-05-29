import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np
import struct
import threading

class TCPIPServer(Node):
    def __init__(self):
        super().__init__('tcpip_server_node')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', 5607))
        self.server_socket.listen(1)
        self.get_logger().info("Waiting for connection...")
        self.client_socket, self.client_address = self.server_socket.accept()
        self.connection = self.client_socket.makefile('wb')
        self.get_logger().info(f"Connected to {self.client_address}")
        self.cap = cv2.VideoCapture(0)
        
        self.server_thread = threading.Thread(target=self.publish_frames)
        self.server_thread.start()

    def publish_frames(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                encoded, buffer = cv2.imencode('.jpg', frame)
                data = np.array(buffer)
                string_data = data.tobytes()
                self.connection.write(struct.pack('<L', len(string_data)))
                self.connection.write(string_data)
                self.connection.flush()

def main(args=None):
    rclpy.init(args=args)
    node = TCPIPServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
