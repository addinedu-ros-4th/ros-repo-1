from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from threading import Thread

import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

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

        self.current_mode = "main" # 초기 모드 설정

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
        self.pixmap = self.pixmap.scaled(self.frame2.width(), self.frame2.height())

        self.update_image_signal.connect(self.update_image)

    def setmain(self):
        self.current_mode = "main"
        self.pushNormal.hide()
        self.mainGroup.show()
        self.registerGroup.hide()
        self.cameraGroup.hide()

    def register(self):
        self.current_mode = "register"
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.show()
        self.cameraGroup.hide()

    def setcamera(self):
        self.current_mode = "commute"
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.hide()
        self.cameraGroup.show()

    @pyqtSlot(np.ndarray, list, bool)
    def update_image(self, cv_img, names, is_register_mode):
        if is_register_mode:
            qt_img = self.cv_to_pixmap(cv_img, self.frame)
            self.frame.setPixmap(qt_img)
        else:
            image = self.add_text_to_image(cv_img, names)
            qt_img = self.cv_to_pixmap(image, self.frame2)
            self.frame2.setPixmap(qt_img)

    def cv_to_pixmap(self, cv_img, frames):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, c = rgb_image.shape
        qimage = QImage(rgb_image.data, w, h, w * c, QImage.Format_RGB888)
        qt_img_scaled = qimage.scaled(frames.width(), frames.height(), Qt.KeepAspectRatio)
        return QPixmap.fromImage(qt_img_scaled)

    def add_text_to_image(self, cv_img, names):
        for i, name in enumerate(names):
            cv2.putText(cv_img, name, (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return cv_img

class ImageSubscriber(Node):
    def __init__(self, ui):
        super().__init__("img_sub")
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

    thread = Thread(target = executor.spin)
    thread.start()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()