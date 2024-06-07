import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np
import struct
import threading
# import queue

class TCPIPServer(Node):
    def __init__(self, ui_callback):
        super().__init__('tcpip_server_node')
        self.ui_callback = ui_callback
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', 5607))
        self.server_socket.listen(1)
        self.get_logger().info("Waiting for connection...")
        
        # self.frame_queue = queue.Queue()
        self.accept_thread = threading.Thread(target=self.accept_connections)
        self.accept_thread.start()

    def accept_connections(self):
        while rclpy.ok():
            try:
                self.client_socket, self.client_address = self.server_socket.accept()
                self.connection = self.client_socket.makefile('wb')
                self.get_logger().info(f"Connected to {self.client_address}")
                
                self.cap = cv2.VideoCapture(0)
                self.publish_frames()
            except Exception as e:
                self.get_logger().error(f"Error accepting connection: {e}")

    def publish_frames(self):
        frame_skip = 5  # 2번째마다 한 번씩 프레임 전송
        frame_count = 0
        try:
            while rclpy.ok():
                ret, frame = self.cap.read()
                if ret:
                    frame_count += 1
                    if frame_count % frame_skip != 0:
                        continue  # 프레임 스킵
                    
                    self.ui_callback(frame)

                    encoded, buffer = cv2.imencode('.jpg', frame)
                    data = np.array(buffer)
                    string_data = data.tobytes()
                    self.connection.write(struct.pack('<L', len(string_data)))
                    self.connection.write(string_data)
                    self.connection.flush()
                else:
                    break
        except Exception as e:
            self.get_logger().error(f"Error publishing frames: {e}")
        finally:
            if self.cap.isOpened():
                self.cap.release()
            if self.client_socket:
                self.client_socket.close()
            if self.connection:
                self.connection.close()
            self.get_logger().info("Connection closed, ready to accept new connection.")

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
