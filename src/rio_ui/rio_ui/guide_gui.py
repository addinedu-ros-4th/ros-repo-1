from PyQt5 import uic, QtCore
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import os
import sys
import time
import numpy as np

from pyzbar.pyzbar import decode
from threading import Thread

import rclpy as rp
from rclpy.executors import MultiThreadedExecutor

import cv2
from ament_index_python.packages import get_package_share_directory

from rio_ui.guide_service import *
# from guide_service import *

# print(ui_file)

# module_path = '/home/wook/amr_ws/project/final/ros-repo-1/src/rio_ui/rio_ui'
# print(module_path)
# if module_path not in sys.path:
#     sys.path.append(module_path)

# resource_path = os.path.join(get_package_share_directory("rio_ui"), "resource_rc.py")
# if resource_path not in sys.path:
#     sys.path.append(resource_path)

# import resource_rc

ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "guide_service.ui")
guide_ui = uic.loadUiType(ui_file)[0]


class GuideGUI(QMainWindow, guide_ui):
    def __init__(self):    
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('안내용 로봇')

        self.current_mode = "main"
        self.pixmap = QPixmap()

        self.pushNormal.show()
        self.setmain()
        self.pushRegister.setEnabled(True)
        # self.pushVisitor.setEnabled(False)
        self.pushQR.setEnabled(True)
        self.pushCommute.setEnabled(True)

        self.pushNormal.clicked.connect(self.setmain)
        self.pushRegister.clicked.connect(self.register)
        self.pushRetake.clicked.connect(self.retake)
        self.pushRegister2.clicked.connect(self.registerinfo)
        self.pushQR.clicked.connect(self.qrcheck)
        self.pushCommute.clicked.connect(self.setcamera)
        self.pushBack.clicked.connect(self.setmain)
        self.pushBack_2.clicked.connect(self.setmain2)

        self.birthEdit.setCalendarPopup(True)
        self.birthEdit.setDateTime(QtCore.QDateTime.currentDateTime())

        self.decode_thread = QRDecodeThread()
        self.camera = Camera()
        self.camera.update_image_signal.connect(self.update_qr_image)
        self.camera.frame_captured_signal.connect(self.decode_thread.decode)
        self.decode_thread.decoded_data_signal.connect(self.handle_decoded_data)

        # self.register_service = RegisterService()

        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.register_service.send_service_request)
        
        self.signals = ROSGuideNodeSignals()
        self.signals.update_image_signal.connect(self.update_image)
        self.signals.face_registration.connect(self.face_registration)

        # self.signals.update_mode_signal = pyqtSignal(str)

    def setmain(self):
        self.current_mode = "main"
        # self.signals.update_mode_signal.emit(self.current_mode)
        self.pushNormal.hide()
        self.mainGroup.show()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        # self.timer.stop()

    def setmain2(self):
        self.current_mode = "main"
        # self.signals.update_mode_signal.emit(self.current_mode)
        self.pushNormal.hide()
        self.mainGroup.show()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        self.camera.stop()
        self.decode_thread.stop()
        # self.timer.stop()        

    def register(self):
        self.current_mode = "register"
        # self.signals.update_mode_signal.emit(self.current_mode)
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.show()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        self.label.setText("카메라상에 얼굴이 잘 인식되도록 위치시켜 주세요")

        # self.guide_service.send_service_request()
        # self.timer.start(2000)

    def qrcheck(self):
        self.current_mode = "qrcheck"
        # self.signals.update_mode_signal.emit(self.current_mode)
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()    
        self.QRGroup.show()
        self.qr_label.setText("카메라 중앙에 qr을 위치시켜 주세요.")
        self.qr_client = QRCheckClient()
        self.camera.start()
        self.decode_thread.start()

    # @pyqtSlot(QPixmap)
    def face_registration(self, landmark_checked):
        if landmark_checked:
            self.label.setText("얼굴 등록이 완료되었습니다")
            print("All required landself.QRGroup.hide()marks are available!")
            saved_face = self.frame.pixmap()
            self.info_registration(saved_face)
        else:
            self.label.setText("카메라상에 얼굴이 잘 인식되도록 위치시켜 주세요")
            print("Not all required landmarks are available!")

    def info_registration(self, saved_face):
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.hide()
        self.registerGroup2.show()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        # self.timer.stop()
        scaled_pixmap = saved_face.scaled(self.frame3.size(), QtCore.Qt.KeepAspectRatio)
        self.frame3.setPixmap(scaled_pixmap)

    def retake(self):
        self.register()

    def registerinfo(self):
        name = self.nameEdit.text()
        birth = self.birthEdit.date().toString("yyyyMMdd")
        office = self.officeEdit.text()
        phone_number = self.phoneEdit.text()
        pixmap = self.frame3.pixmap()
        user_face = self.image_to_binary(pixmap)

        data = {
            "name": name,
            "birth": birth,
            "office": office,
            "phone_number": phone_number,
            "user_face": user_face
        }

    def image_to_binary(self, pixmap):
        if pixmap:
            qimage = pixmap.toImage()
            buffer = QBuffer()
            buffer.open(QIODevice.ReadWrite)
            qimage.save(buffer, "JPEG")
            img_data = buffer.data()
            buffer.close()

            return img_data
        else:
            print("No pixmap found in frame3")

    def setcamera(self):
        self.current_mode = "commute"
        # self.signals.update_mode_signal.emit(self.current_mode)
        self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.hide()
        self.QRGroup.hide()
        self.cameraGroup.show()
        self.QRGroup.hide()
        # self.timer.stop()

    # @pyqtSlot(np.ndarray, list, bool)
    def update_image(self, cv_img, names):
        is_register_mode = (self.current_mode == "register")
        print("is_register_mode : ", is_register_mode)
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
    
    @pyqtSlot(QImage)
    def update_qr_image(self, image):
        self.QRframe.setPixmap(QPixmap.fromImage(image)) 

    @pyqtSlot(str)
    def handle_decoded_data(self, decoded_data):
        print(f"Decoded Data: {decoded_data}")
        self.qr_client.send_request(decoded_data)
        self.qr_client.destroy_node()
        rp.shutdown()
        self.qr_client = None

    def closeEvent(self, event):
        self.camera.stop()
        self.decode_thread.stop()
        if self.qr_client:
            self.qr_client.destroy_node()
            rp.shutdown()
        self.camera.wait()
        self.decode_thread.wait()
        event.accept()        
    
class Camera(QThread):
    frame_captured_signal = pyqtSignal(np.ndarray)
    update_image_signal = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            self.running = False
        else:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.running = True        
    
    def run(self):
        while self.running:
            self.cap.grab()
            ret, frame = self.cap.read()
            if ret:
                self.frame_captured_signal.emit(frame) 
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                p = convert_to_qt_format.scaled(w, h , aspectRatioMode=1)
                self.update_image_signal.emit(p)
            time.sleep(0.05)
    
    def stop(self):
        self.running = False
        self.cap.release()
        self.quit()

class QRDecodeThread(QThread):
    decoded_data_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = False
        # self.processed_data = set()

    def run(self):
        self.running = True
        while self.running:
            time.sleep(0.01)

    def decode(self, frame):
        decoded_objects = decode(frame)
        for obj in decoded_objects:
            decoded_data = obj.data.decode("utf-8")
            self.decoded_data_signal.emit(decoded_data)

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

def main():
    rp.init()

    app = QApplication(sys.argv)

    myWindows = GuideGUI()
    myWindows.show()

    signals = myWindows.signals
    executor = MultiThreadedExecutor()

    image_subscriber = ImageSubscriber(signals)
    # myWindows.current_mode = "wooktest"
    # image_subscriber.mode = myWindows.current_mode
    # print("여기", image_subscriber.mode)
    face_name_subscriber = FacenameSubscriber(signals, image_subscriber)
    face_details_subscriber = FacelandmarkSubscriber(signals)

    # myWindows.signals.update_mode_signal.connect(lambda mode: setattr(image_subscriber, 'mode', mode))

    executor.add_node(image_subscriber)
    executor.add_node(face_name_subscriber)
    executor.add_node(face_details_subscriber)

    thread = Thread(target=executor.spin)
    thread.start()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()