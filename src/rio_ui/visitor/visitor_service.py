import sys
import rclpy as rp
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import uic
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import numpy as np

from_class = uic.loadUiType("visitor.ui")[0]

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('안내용 로봇')

        self.pushNormal.show()
        self.mainGroup.hide()
        self.cameraGroup.hide()
        self.pushRegister.setEnabled(False)
        self.pushQR.setEnabled(False)
        self.pushGuide.setEnabled(False)
        self.pushCommute.setEnabled(True)

        self.pushNormal.clicked.connect(self.setmain)
        self.pushCommute.clicked.connect(self.setcamera)

        self.pixmap = QPixmap()
        self.pixmap = self.pixmap.scaled(self.frame.width(), self.frame.height())

        self.ros_receiver = ROSImageReceiver()
        self.ros_receiver.update_image.connect(self.update_image)

    def setmain(self):
        self.pushNormal.hide()
        self.mainGroup.show()
        self.cameraGroup.hide()
        self.ros_receiver.stop()

    def setcamera(self):
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.cameraGroup.show()
        self.ros_receiver.start()

    def closeEvent(self, event):
        self.ros_receiver.stop()
        event.accept()

    def update_image(self, cv_img, names):
        image = self.add_text_to_image(cv_img, names)
        qt_img = self.cv_to_pixmap(image)
        self.frame.setPixmap(qt_img)

    def cv_to_pixmap(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, c = rgb_image.shape
        qimage = QImage(rgb_image.data, w, h, w * c, QImage.Format_RGB888)
        qt_img_scaled = qimage.scaled(self.frame.width(), self.frame.height(), Qt.KeepAspectRatio)
        return QPixmap.fromImage(qt_img_scaled)

    def add_text_to_image(self, cv_img, names):
        for i, name in enumerate(names):
            cv2.putText(cv_img, name, (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return cv_img
    
class ROSImageReceiver(QThread):
    update_image = pyqtSignal(np.ndarray, list)

    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.image = None
        self.names = []
        self.node = None

    def run(self):
        rp.init()
        self.node = rp.create_node('Image_Receiver')
        self.image_sub = self.node.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.names_sub = self.node.create_subscription(String, '/face_names', self.names_callback, 10)
        rp.spin(self.node)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.update_image.emit(cv_image, self.names)
        except CvBridgeError as e:
            print(e)

    def names_callback(self, data):
        self.names = data.data.split(',')

    def stop(self):
        if self.node is not None:
            self.node.destroy_node()
        if rp.ok():
            rp.shutdown()
        self.wait()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())