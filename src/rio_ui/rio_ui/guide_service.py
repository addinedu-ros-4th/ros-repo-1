from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from threading import Thread

import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

import numpy as np
import os
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError

# ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "guide_service.ui")
ui_file = "ui/guide_service.ui"
guide_ui = uic.loadUiType(ui_file)[0]

class WindowClass(QMainWindow, guide_ui):
    update_image_signal = pyqtSignal(np.ndarray, list, bool)

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('안내용 로봇')

        self.current_mode = "main"

        self.pushNormal.show()
        self.mainGroup.hide()
        self.cameraGroup.hide()
        self.pushRegister.setEnabled(True)
        self.pushVisitor.setEnabled(False)
        self.pushQR.setEnabled(False)
        self.pushCommute.setEnabled(True)

        self.pushNormal.clicked.connect(self.setmain)
        self.pushRegister.clicked.connect(self.register)
        self.pushCommute.clicked.connect(self.setcamera)
        self.pushBack.clicked.connect(self.setmain)

        self.pixmap = QPixmap()
        # self.pixmap = self.pixmap.scaled(self.frame2.width(), self.frame2.height())

        self.update_image_signal.connect(self.update_image)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.send_service_request)

        self.create_service_client()

    def create_service_client(self):
        self.node = rp.create_node('pyqt_service_client')
        self.client = self.node.create_client(SetBool, 'register_service')

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

    def setmain(self):
        self.current_mode = "main"
        self.pushNormal.hide()
        self.mainGroup.show()
        self.registerGroup.hide()
        self.cameraGroup.hide()
        self.timer.stop()

    def register(self):
        self.current_mode = "register"
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.show()
        self.cameraGroup.hide()
        self.label.setText("카메라상에 얼굴이 잘 인식되도록 위치시켜 주세요")

        self.send_service_request()
        self.timer.start(2000)

    def setcamera(self):
        self.current_mode = "commute"
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.hide()
        self.cameraGroup.show()
        self.timer.stop()

    def publish_command(self, command):
        command_msg = String()
        command_msg.data = command
        self.command_pub.publish(command_msg)

    @pyqtSlot(np.ndarray, list, bool)
    def update_image(self, cv_img, names, is_register_mode):
        if is_register_mode:
            qt_img = self.cv_to_pixmap(cv_img, self.frame.width(), self.frame.height())
            self.frame.setPixmap(qt_img)
        else:
            image = self.add_text_to_image(cv_img, names)
            qt_img = self.cv_to_pixmap(image, self.frame2.width(), self.frame2.height())
            self.frame2.setPixmap(qt_img)

    def cv_to_pixmap(self, cv_img, width, height):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, c = rgb_image.shape
        qimage = QImage(rgb_image.data, w, h, w * c, QImage.Format_RGB888)
        qt_img_scaled = qimage.scaled(width, height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(qt_img_scaled)

    def add_text_to_image(self, cv_img, names):
        for i, name in enumerate(names):
            cv2.putText(cv_img, name, (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return cv_img

class ImageSubscriber(Node):
    def __init__(self, ui):
        super().__init__("image_sub")
        self.ui = ui
        self.bridge = CvBridge()
        self.image = None
        self.names = []
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

    def image_callback(self, data):
        is_register_mode = (self.ui.current_mode == "register")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.ui.update_image_signal.emit(cv_image, self.names, is_register_mode)
        except CvBridgeError as e:
            print(e)

    def update_names(self, names):
        self.names = names

class FacenameSubscriber(Node):
    def __init__(self, ui, image_subscriber):
        super().__init__("face_name_sub")
        self.ui = ui
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
    def __init__(self, ui):
        super().__init__("face_landmarks_sub")
        self.ui = ui
        self.required_landmarks = ["chin", "left_eyebrow", "right_eyebrow", "nose_bridge", "nose_tip", "left_eye", "right_eye", "top_lip", "bottom_lip"]
        self.landmarks_sub = self.create_subscription(
            String,
            '/face_landmarks',
            self.landmarks_callback,
            10
        )
        self.timeout_timer = QTimer()
        self.timeout_timer.setSingleShot(True)
        self.timeout_timer.timeout.connect(self.handle_timeout)

    def landmarks_callback(self, data):
        try :
            face_landmarks = eval(data.data)
            if self.check_required_landmarks(face_landmarks):
                self.ui.label.setText("얼굴 등록이 완료되었습니다")
                print("All required landmarks are available!")
                self.timeout_timer.stop()
            else:
                self.ui.label.setText("카메라상에 얼굴이 잘 인식되도록 위치시켜 주세요")
                print("Not all required landmarks are available!")
                self.timeout_timer.start(5000)
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
        
    def handle_timeout(self):
        self.ui.label.setText("얼굴의 특징점을 찾을 수 없습니다")

def main():

    rp.init()
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    image_subscriber = ImageSubscriber(myWindows)
    executor.add_node(image_subscriber)

    face_name_subscriber = FacenameSubscriber(myWindows, image_subscriber)
    executor.add_node(face_name_subscriber)

    face_details_subscriber = FacelandmarkSubscriber(myWindows)
    executor.add_node(face_details_subscriber)

    thread = Thread(target = executor.spin)
    thread.start()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()