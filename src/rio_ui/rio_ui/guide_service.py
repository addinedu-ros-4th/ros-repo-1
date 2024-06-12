import rclpy as rp
from rclpy.node import Node

from std_srvs.srv import SetBool
from rio_ui_msgs.srv import QRCheck
from sensor_msgs.msg import Image
from std_msgs.msg import String

from PIL import Image as pim

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import socket
import threading
import cv2
import struct
import numpy as np
import time
from cv_bridge import CvBridge, CvBridgeError

class RegisterService(Node):
    def __init__(self):
        super().__init__('register_service')
        self.create_service_client()

    def create_service_client(self):
        self.client = self.create_client(SetBool, 'landmark_client')

        while not self.client.wait_for_service(timeout_sec=1.0):
            print('Service not available, waiting...')

    def send_service_request(self):
        if self.client.service_is_ready():
            req = SetBool.Request()
            req.data = True
            future = self.client.call_async(req)
            future.add_done_callback(self.handle_service_response)
        else:
            print('Service is not ready')

    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                print(f'Success: {response.message}')
            else:
                print(f'Failure: {response.message}')
        except Exception as e:
            print(f'Service call failed: {e}')

class QRCheckClient(Node):
    def __init__(self, signals):
        super().__init__('qr_check_client')
        # self.cli = self.create_client(QRCheck, 'qr_check_1')
        self.cli = self.create_client(QRCheck, 'qr_check')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = QRCheck.Request()
        self.signals = signals
    
    def send_request(self, decoded_data):
        self.request.hashed_data = decoded_data
        future = self.cli.call_async(self.request)
        # rp.spin_until_future_complete(self, future)
        # future.add_done_callback(self.handle_response)
        rp.spin_until_future_complete(self, future)
        self.handle_response(future)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response {response.success}, {response.message}')
            if response.success == "True":
                self.signals.qr_service_signal.emit(response.success)
            else:
                self.signals.qr_service_signal.emit(response.success)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')



class ROSGuideNodeSignals(QObject):
    update_image_signal = pyqtSignal(np.ndarray)
    face_registration = pyqtSignal(bool)
    qr_service_signal = pyqtSignal(bool)

class ImageSubscriber(Node):
    def __init__(self, signals):
        super().__init__("image_sub")
        self.signals = signals
        self.bridge = CvBridge()
        self.image = None
        self.names = []
        self.img_sub = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )
        self.mode = ""

    #     self.signals.update_mode_signal.connect(self.set_mode)

    # def set_mode(self, mode):
    #     self.mode = mode

    def image_callback(self, data):
        # is_register_mode = (self.mode == "register")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.signals.update_image_signal.emit(cv_image, self.names)
        except CvBridgeError as e:
            print(e)

    def update_names(self, names):
        self.names = names

class FacenameSubscriber(Node):
    def __init__(self, signals, image_subscriber):
        super().__init__("face_name_sub")
        self.signals = signals
        self.image_subscriber = image_subscriber
        self.names_sub = self.create_subscription(
            String,
            '/face_names', 
            self.names_callback, 
            10
        )
        
    def names_callback(self, data):
        try:
            names = data.data.split(',')
            self.image_subscriber.update_names(names)
        except CvBridgeError as e:
            print(e)   

class FacelandmarkSubscriber(Node):
    def __init__(self, signals):
        super().__init__("face_landmarks_sub")
        self.signals = signals
        self.required_landmarks = ["chin", "left_eyebrow", "right_eyebrow", "nose_bridge", "nose_tip", "left_eye", "right_eye", "top_lip", "bottom_lip"]
        self.landmarks_sub = self.create_subscription(
            String,
            '/face_landmarks',
            self.landmarks_callback,
            10
        )
        print("FacelandmarkSubscriber initialized")

    def landmarks_callback(self, data):
        try:
            face_landmarks = eval(data.data)
            landmark_checked = self.check_required_landmarks(face_landmarks)
                
            self.signals.face_registration.emit(landmark_checked)
                
        except Exception as e:
            print(e)

    def check_required_landmarks(self, face_landmarks):
        if len(face_landmarks) > 0:
            return True
        else:
            return False

class ImageUpdater(QObject):
    image_updated = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()

    def update_image(self, image):
        # 이미지 업데이트 로직
        self.image_updated.emit(image)

class GuideTCPIPServer(QObject):
    def __init__(self, image_updater):
        super().__init__()
        self.image_updater = image_updater
        self.image_updater.image_updated.connect(self.publish_frames)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('192.168.0.24', 5604))
        self.server_socket.listen(1)
        print("waiting connection")
        
        self.client_socket = None
        self.connection = None
        self.accept_thread = threading.Thread(target=self.accept_connections)
        self.accept_thread.start()

    def accept_connections(self):
        try:
            while True:
                self.client_socket, self.client_address = self.server_socket.accept()
                self.connection = self.client_socket.makefile('wb')
                
        except Exception as e:
            print(f"accept_connection :{e}")
    def publish_frames(self, frame):
        if self.connection:
            try:
                encoded, buffer = cv2.imencode('.jpg', frame)
                data = np.array(buffer)
                string_data = data.tobytes()
                self.connection.write(struct.pack('<L', len(string_data)))
                self.connection.write(string_data)
                self.connection.flush()
                    
            except Exception as e:
                self.connection.close()
                self.client_socket = None
                self.connection = None
                print(f"publish_frames{e}")
                
    def stop_server(self):
        if self.connection:
            self.connection.close()
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()
        print("Server is closed")

class UserTCPIPClientNode(QObject):
    image_received = pyqtSignal(np.ndarray)

    def __init__(self, ip_address, port):
        super().__init__()
        self.client_socket = None
        self.connection = None
        self.connect_to_server(ip_address, port)
        self.receive_thread = threading.Thread(target=self.receive_video)
        self.receive_thread.start()

            
    def connect_to_server(self, ip_address, port):
        while True:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((ip_address, port))
                self.connection = self.client_socket.makefile('rb')
                print(f"Connected to server at {ip_address}:{port}")
                break
            except Exception as e:
                print(f"Failed to connect to server at {ip_address}:{port}: {e}")
                time.sleep(5)

    def receive_video(self):
        while True:
            try:
                while True:
                    packed_msg_size = self.connection.read(struct.calcsize('<L'))
                    
                    if not packed_msg_size:
                        break
                    msg_size = struct.unpack('<L', packed_msg_size)[0]
                    frame_data = self.connection.read(msg_size)
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    
                    if frame is not None and frame.size != 0:
                        self.image_received.emit(frame)  # 이미지 수신 시그널 방출
                    else:
                        print("Invalid frame received")
            except Exception as e:
                print(f"Error: {e}")


def main(args=None):
    rp.init(args=args)
    guide_service = RegisterService()
    rp.spin(guide_service)
    rp.shutdown()

if __name__ == '__main__':
    main()