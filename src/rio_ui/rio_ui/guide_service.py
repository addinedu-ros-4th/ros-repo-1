import rclpy as rp
from rclpy.node import Node

from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from std_msgs.msg import String

from PIL import Image as pim

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from rio_ui.guide_gui import *

class GuideService(Node):
    def __init__(self):
        super().__init__('guide_service')
        self.create_service_client()

    def create_service_client(self):
        self.client = self.create_client(SetBool, 'register_service')

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

class ROSGuideNodeSignals(QObject):
    update_image_signal = pyqtSignal(np.ndarray, list)
    face_registration = pyqtSignal(bool)

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
            landmarks_dict = face_landmarks[0]
            for landmark in self.required_landmarks:
                if landmark not in landmarks_dict:
                    return False
            return True
        else:
            return False

def main(args=None):
    rp.init(args=args)
    guide_service = GuideService()
    rp.spin(guide_service)
    rp.shutdown()

if __name__ == '__main__':
    main()