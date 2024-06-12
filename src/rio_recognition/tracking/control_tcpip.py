import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import socket
import struct
import time
import cv2
import numpy as np
from tracker_tcpip import ObjectTracker

class TrackingController(Node):
    def __init__(self, object_tracker):
        super().__init__('obj_tracking_controller')
        self.publisher_ = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.height = 480 
        self.width = 640 
        self.is_control_active = True
        self.alpha = 0.1  # LPF coefficient for area
        self.alpha_angle = 0.1  # LPF coefficient for angular speed
        self.filtered_obj_area = 0.0
        self.filtered_angular_speed = 0.0
        self.object_tracker = object_tracker

    def timer_callback(self):
        if self.is_control_active:
            distance, obj_area = self.get_box_info()

            if distance is None or obj_area is None:
                return
            
            self.filtered_obj_area = self.alpha * obj_area + (1 - self.alpha) * self.filtered_obj_area

            if -10 < distance < 10 and obj_area <= 100000:
                self.is_control_active = False
                self.stop_control()
            else:
                twist = Twist()
                self.control_speed(twist, distance, self.filtered_obj_area)
                self.control_direction(twist, distance)
                self.publisher_.publish(twist)
        else:
            print("control_deactivate")

    def get_box_info(self):
        distance = self.object_tracker.track_distance
        obj_area = self.object_tracker.track_area
        return distance, obj_area
    
    def control_direction(self, twist, distance):
        max_angular_speed = 0.5
        min_angular_speed = 0.0
        angular_speed = min_angular_speed + (max_angular_speed - min_angular_speed) * abs(distance) / 320.0
        self.filtered_angular_speed = self.alpha_angle * angular_speed + (1 - self.alpha_angle) * self.filtered_angular_speed

        if self.object_tracker.found_match:
            if distance < -10:
                twist.angular.z = self.filtered_angular_speed
            elif distance > 10:
                twist.angular.z = -self.filtered_angular_speed
        else:
            last_box = self.object_tracker.last_known_position
            if ((last_box[2] + last_box[0]) / 2) > (self.width/2):
                twist.angular.z = -self.filtered_angular_speed
            else:
                twist.angular.z = self.filtered_angular_speed

        self.get_logger().info(f'Controlling direction: distance={distance}, angular.z={twist.angular.z}')

    def control_speed(self, twist, distance, obj_area):
        max_speed = -0.1
        min_speed = -0.0
        max_area = 300000
        min_area = 100000

        if obj_area > max_area:
            speed = max_speed
        elif obj_area < min_area:
            speed = min_speed
        else:
            speed = min_speed + (max_speed - min_speed) * (max_area - obj_area) / (max_area - min_area)

        twist.linear.x = speed   
        self.get_logger().info(f'Controlling robot: distance={distance}, obj_area={obj_area}, speed={speed}' )

    def stop_control(self):
        twist = Twist()
        twist.linear.x = -0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Target conditions met')
        self.is_control_active = True

class TCPIPClientNode(Node):
    def __init__(self, ip_address, model_path, object_name, tracking_controller):
        super().__init__('tcpip_client_node')
        self.client_socket = None
        self.connection = None
        self.connect_to_server(ip_address)
        self.ip_address = ip_address
        self.model_path = model_path
        self.object_name = object_name
        self.tracking_controller = tracking_controller

        self.receive_thread = threading.Thread(target=self.receive_video)
        self.receive_thread.start()

        self.object_tracker = ObjectTracker(self.model_path, self.object_name)
        self.object_tracker.start()
        self.tracking_controller.object_tracker = self.object_tracker

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
                packed_msg_size = self.connection.read(struct.calcsize('<L'))
                msg_size = struct.unpack('<L', packed_msg_size)[0]
                frame_data = self.connection.read(msg_size)
                frame = np.frombuffer(frame_data, dtype=np.uint8)
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                if frame is not None and frame.size != 0:
                    self.object_tracker.add_frame(frame)
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
        # self.executor_thread.join()
        self.object_tracker.running = False
        self.object_tracker.join()

def main(args=None):
    rclpy.init(args=args)

    # ip_address = "192.168.0.63"
    ip_address = "172.30.1.44"
    model_path = 'best.pt'
    object_name = 'wook'  # db에서 제공받은 이름

    object_tracker = ObjectTracker(model_path, object_name)
    tracking_controller = TrackingController(object_tracker)

    tcpip_client_node = TCPIPClientNode(ip_address, model_path, object_name, tracking_controller)

    executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(object_tracker)
    executor.add_node(tracking_controller)
    executor.add_node(tcpip_client_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    tcpip_client_node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
