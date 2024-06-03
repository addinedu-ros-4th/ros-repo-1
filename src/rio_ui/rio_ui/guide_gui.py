from PyQt5 import uic, QtCore
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import os
import sys
import time

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

        # self.pushNormal.show()
        self.setmain()
        self.pushRegister.setEnabled(True)
        # self.pushVisitor.setEnabled(False)
        self.pushQR.setEnabled(True)
        self.pushCommute.setEnabled(True)

        # self.pushNormal.clicked.connect(self.setmain)
        self.pushRegister.clicked.connect(self.register)
        self.pushRetake.clicked.connect(self.retake)
        self.pushRegister2.clicked.connect(self.registerinfo)
        self.pushQR.clicked.connect(self.qrcheck)
        self.pushCommute.clicked.connect(self.setcamera)
        self.pushBack.clicked.connect(self.setmain)
        self.pushBack_2.clicked.connect(self.setmain)

        self.birthEdit.setCalendarPopup(True)
        self.birthEdit.setDateTime(QtCore.QDateTime.currentDateTime())

        self.camera = Camera(self)
        self.camera.daemon = True
        self.camera.update.connect(self.read_qr)

        # self.guide_service = GuideService()

        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.guide_service.send_service_request)

        self.signals = ROSGuideNodeSignals()
        self.signals.update_image_signal.connect(self.update_image)
        self.signals.face_registration.connect(self.face_registration)

        # self.signals.update_mode_signal = pyqtSignal(str)

    def setmain(self):
        self.current_mode = "main"
        # self.signals.update_mode_signal.emit(self.current_mode)
        # self.pushNormal.hide()
        self.mainGroup.show()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        # self.timer.stop()

    def register(self):
        self.current_mode = "register"
        # self.signals.update_mode_signal.emit(self.current_mode)
        # self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.show()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        self.label.setText("카메라상에 얼굴이 잘 인식되도록 위치시켜 주세요")

        # self.guide_service.send_service_request()
        # self.timer.start(2000)

    def cameraStart(self):
        self.camera.running = True
        self.camera.start()
        self.video = cv2.VideoCapture(0)
        self.last_message_time = time.time()

    def cameraStop(self):
        self.camera.running = False
        self.video.release()


    def qrcheck(self):
        self.current_mode = "qrcheck"
        # self.signals.update_mode_signal.emit(self.current_mode)
        # self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()    
        self.qr_label.setText("카메라 중앙에 qr을 위치시켜 주세요.")
        self.cameraStart()

    def read_qr(self):
        # last_message_time = time.time()
        message_interval = 3  # 메시지 출력 간격 (초)

        ret, frame = self.video.read()

        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            h, w, c = frame.shape
            qimage = QImage(frame.data, w, h, w*c, QImage.Format_RGB888)
            pixmap = qimage.scaled(self.QRframe.width(), self.QRframe.height(), Qt.KeepAspectRatio)
            
            self.QRframe.setPixmap(QPixmap.fromImage(pixmap))
            qr_codes = decode(frame)
            if qr_codes:
                for qr_code in qr_codes:
                    qr_data = qr_code.data.decode('utf-8')
                    print("QR Code detected", qr_data)
                    self.qr_label.setText("checked!")

            else:
                if time.time() - self.last_message_time >= message_interval:
                    self.qr_label.setText("QR 코드를 찾을 수 없습니다. 다시 시도 중...")
                    self.last_message_time = time.time()  


    # @pyqtSlot(QPixmap)
    def face_registration(self, landmark_checked):
        if landmark_checked:
            self.label.setText("얼굴 등록이 완료되었습니다")
            print("All required landmarks are available!")
            saved_face = self.frame.pixmap()
            self.info_registration(saved_face)
        else:
            self.label.setText("카메라상에 얼굴이 잘 인식되도록 위치시켜 주세요")
            print("Not all required landmarks are available!")

    def info_registration(self, saved_face):
        # self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.hide()
        self.registerGroup2.show()
        self.cameraGroup.hide()
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
        # self.pushNormal.hide()
        self.mainGroup.hide()
        self.registerGroup.hide()
        self.QRGroup.hide()
        self.cameraGroup.show()
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
    
class Camera(QThread):
    update = pyqtSignal()

    def __init__(self, sec=0):
        super().__init__()
        self.running = True
    
    def run(self):
        # self.last_message_time = time.time()
        # message_interval = 3  # 메시지 출력 간격 (초)
        while self.running == True:
            self.update.emit()
            time.sleep(0.1)

    def stop(self):
        self.running = False

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