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
# from guide_service3 import *

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
        # self.mainGroup.hide()
        # self.QRGroup.hide()
        # self.registerGroup.hide()
        # self.registerGroup2.hide()
        # self.cameraGroup.hide()
        # self.video_confGroup.hide()

        self.setmain()
        self.QR_bt.setEnabled(True)
        self.tenant_group_bt.setEnabled(True)

        # self.pushRegister.setEnabled(True)
        # self.pushQR.setEnabled(True)
        # self.pushCommute.setEnabled(True)

        # self.pushNormal.clicked.connect(self.setmain)

        self.pushRegister.clicked.connect(self.register)
        self.pushRetake.clicked.connect(self.retake)
        self.pushRegister2.clicked.connect(self.registerinfo)
        self.QR_bt.clicked.connect(self.qrcheck)
        self.tenant_group_bt.clicked.connect(self.tenant_service)
        self.pushCommute.clicked.connect(self.setcamera)
        self.pushBack.clicked.connect(self.setmain)
        self.pushBack_2.clicked.connect(self.setmain2)
        self.pushBack_3.clicked.connect(self.setmain)
        self.video_meet_bt.clicked.connect(self.video_conf)
        self.connect_bt.clicked.connect(self.connect_meeting)

        self.birthEdit.setCalendarPopup(True)
        self.birthEdit.setDateTime(QtCore.QDateTime.currentDateTime())

        self.camera = None
        self.decode_thread = None
        self.tcpip_server = None

        self.namelist = ["ho", "wook", "kyu", "bin"] # DB에 등록된 사람 이름 리스트
        self.is_working = True # 출/퇴근 상태 flag -> DB

        # self.register_service = RegisterService()

        self.timer = QTimer(self)
        # self.timer.timeout.connect(self.register_service.send_service_request)
        
        self.signals = ROSGuideNodeSignals()
        self.signals.update_name_signal.connect(self.update_name)
        self.signals.face_registration.connect(self.face_registration)
        self.signals.qr_service_signal.connect(self.qrcheck_success)

        self.image_updater = ImageUpdater()

        # self.guideserver = GuideVideoServer('192.168.0.10', 5600, self.video_conf_frame)
        # threading.Thread(target=self.guideserver.receive_images, daemon=True).start()

        # self.info_registrator = UserRegisterInfo()

        # self.signals.update_mode_signal = pyqtSignal(str)

    def setmain(self):
        self.current_mode = "main"
        # self.signals.update_mode_signal.emit(self.current_mode)
        # self.pushNormal.hide()
        self.maingroup.show()
        self.tenant_service_group.hide()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        self.video_confGroup.hide()
        # if self.camera is not None and self.camera.isRunning():
        #     self.camera.stop()
        #     self.decode_thread.stop()
        #     self.camera = None
        #     self.decode_thread = None

        # self.timer.stop()  

    def setmain2(self):
        self.current_mode = "main"
        # self.signals.update_mode_signal.emit(self.current_mode)
        self.QRframe.clear()
        # self.pushNormal.hide()
        self.maingroup.show()
        self.tenant_service_group.hide()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        self.video_confGroup.hide()
        if self.camera is not None and self.camera.isRunning():
            self.camera.stop()
            self.decode_thread.stop()
            self.camera = None
            self.decode_thread = None

        self.timer.stop()  

    def tenant_service(self):
        self.QRframe.clear()
        self.maingroup.hide()
        self.tenant_service_group.show()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        self.video_confGroup.hide()

    def register(self):
        self.current_mode = "register"
        # self.signals.update_mode_signal.emit(self.current_mode)
        # self.pushNormal.hide()
        self.maingroup.hide()
        self.tenant_service_group.hide()
        self.registerGroup.show()
        self.registerGroup2.hide()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        self.video_confGroup.hide()
        self.label.setText("카메라상에 얼굴이 잘 인식되도록 위치시켜 주세요")
        
        self.register_cam = RegisterCam()
        self.register_cam.daemon = True
        self.camera_start()
        self.register_cam.update.connect(self.update_camera)

        self.register_service = RegisterService()
        self.timer.timeout.connect(self.register_service.send_service_request)

        self.guide_service.send_service_request()
        self.timer.start(2000)

    def qrcheck(self):
        self.current_mode = "qrcheck"
        # self.signals.update_mode_signal.emit(self.current_mode)
        # self.pushNormal.hide()
        self.QRGroup.show()
        self.tenant_service_group.hide()
        # self.mainGroup.hide()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()    
        self.QRGroup.show()
        self.video_confGroup.hide()
        self.qr_label.setText("카메라 중앙에 qr을 위치시켜 주세요.")
        self.qr_client = QRCheckClient(self.signals)
        self.decode_thread = QRDecodeThread()
        self.camera = Camera()
        self.camera.start()
        self.decode_thread.start()
        self.camera.update_image_signal.connect(self.update_qr_image)
        self.camera.frame_captured_signal.connect(self.decode_thread.decode)
        self.decode_thread.decoded_data_signal.connect(self.handle_decoded_data)
    
    def qrcheck_success(self, success):
        if success:
            self.qr_label.setText("인증 완료")
            if self.camera is not None and self.camera.isRunning():
                self.decode_thread.last_frame_signal.connect(self.display_last_frame)
                self.camera.stop()
                self.decode_thread.stop()
                self.camera = None
                self.decode_thread = None
        else:
            self.qr_label.setText("인증 실패, 다시 QR을 위치시켜 주세요.")
    
    def display_last_frame(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_qt_format.scaled(w, h , aspectRatioMode=1)
        self.QRframe.setPixmap(QPixmap.fromImage(p))

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
        # self.pushNormal.hide()
        self.maingroup.hide()
        self.tenant_service_group.hide()
        self.registerGroup.hide()
        self.registerGroup2.show()
        self.cameraGroup.hide()
        self.QRGroup.hide()
        self.video_confGroup.hide()
        self.timer.stop()
        scaled_pixmap = saved_face.scaled(self.frame3.size(), QtCore.Qt.KeepAspectRatio)
        self.frame3.setPixmap(scaled_pixmap)

    def retake(self):
        self.register()
        self.frame3.clear()

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

        # self.info_registrator.registerinfo(data)

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
        self.maingroup.hide()
        self.tenant_service_group.hide()
        self.registerGroup.hide()
        self.QRGroup.hide()
        self.cameraGroup.show()
        self.QRGroup.hide()
        self.video_confGroup.hide()
        self.timer.stop()

        self.commute_cam = CommuteCam()
        self.commute_cam.daemon = True
        self.camera_start()
        self.commute_cam.update.connect(self.update_camera)

    def update_name(self, names):
        self.name = names[0]
        print('update_name_signal')
        print(self.name, self.current_mode)
        if self.name in self.namelist and self.current_mode == "commute":
            if self.is_working:
                self.commute_label.setText(f"{names[0]}님 출근 처리 완료되샸습니다!!")
                self.is_working = False
            else:
                self.commute_label.setText(f"{names[0]}님 퇴근 처리 되셨습니다!!")
                self.is_working = True
            time.sleep(5) 
            self.camera_stop()
        elif names[0] == "Unknown" or names[0] =="":
            self.commute_label.setText("얼굴 정면이 보이게 카메라를 바라봐주세요!!")

    # @pyqtSlot(np.ndarray, list, bool)
    # def update_image(self, qt_img):
    #     # is_register_mode = (self.current_mode == "register")
    #     # print("is_register_mode : ", is_register_mode)
    #     # if is_register_mode:
    #     if self.current_mode == "register":
    #         qt_img = self.cv_to_pixmap(qt_img, self.frame.width(), self.frame.height())
    #         self.frame.setPixmap(qt_img)
    #     # elif self.current_mode == "qrcheck":
    #     #     qt_img = self.cv_to_pixmap(cv_img, self.frame.width(), self.frame.height())
    #     #     self.frame.setPixmap(qt_img)
    #     else:
    #         image = self.add_text_to_image(qt_img, self.names)
    #         qt_img = self.cv_to_pixmap(image, self.frame2.width(), self.frame2.height())
    #         self.frame2.setPixmap(qt_img)
        

    def cv_to_pixmap(self, cv_img, width, height):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, c = rgb_image.shape
        qimage = QImage(rgb_image.data, w, h, w * c, QImage.Format_RGB888)
        qt_img_scaled = qimage.scaled(width, height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(qt_img_scaled)

    # def add_text_to_image(self, cv_img, names):
    #     if names is not None:
    #         for i, name in enumerate(names):
    #             cv2.putText(cv_img, name, (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    #     return cv_img
    
    @pyqtSlot(QImage)
    def update_qr_image(self, image):
        self.QRframe.setPixmap(QPixmap.fromImage(image)) 

    @pyqtSlot(str)
    def handle_decoded_data(self, decoded_data):
        self.qr_client.send_request(decoded_data)

    def video_conf(self):
        self.current_mode = "conference"
        # self.pushNormal.hide()
        self.maingroup.hide()
        self.tenant_service_group.hide()
        self.registerGroup.hide()
        self.registerGroup2.hide()
        self.cameraGroup.hide()    
        self.QRGroup.hide()
        self.video_confGroup.show()
        self.mycam = MyCam()
        self.mycam.daemon = True
        self.camera_start()
        self.mycam.update.connect(self.update_camera)

    def camera_start(self):

        if self.current_mode == "conference":
            self.mycam.running = True
            self.mycam.start()
        elif self.current_mode == "register":
            self.register_cam.running = True
            self.register_cam.start()
        elif self.current_mode == "commute":
            self.commute_cam.running = True
            self.commute_cam.start()

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("cannot open camera")
            return
        if self.tcpip_server is None:
            self.tcpip_server = GuideTCPIPServer(self.image_updater)
    
    def camera_stop(self):

        if self.current_mode == "conference":
            self.mycam.running = False
        elif self.current_mode == "register":
            self.register_cam.running = False
        elif self.current_mode == "commute":
            self.commute_cam.running = False

        self.cap.release()

    def update_camera(self):

        ret, image = self.cap.read()
        if ret:
            if self.current_mode == "conference":
                qt_img_scaled = self.cv_to_pixmap(image, self.video_conf_frame_2.width(), self.video_conf_frame_2.height())
                self.video_conf_frame_2.setPixmap(qt_img_scaled)
            elif self.current_mode == "register":
                qt_img_scaled = self.cv_to_pixmap(image, self.frame.width(), self.frame.height())
                self.frame.setPixmap(qt_img_scaled)
            elif self.current_mode == "commute":
                # image = self.add_text_to_image(image, names)
                qt_img_scaled = self.cv_to_pixmap(image, self.frame2.width(), self.frame2.height())
                self.frame2.setPixmap(qt_img_scaled)
            
            self.image_updater.update_image(image)

    def connect_meeting(self):
        self.client_node = UserTCPIPClientNode('192.168.0.48', 5608)
        self.client_node.image_received.connect(self.update_somebody_cam)

    @pyqtSlot(np.ndarray)
    def update_somebody_cam(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, c = rgb_image.shape
        qimage = QImage(rgb_image.data, w, h, w * c, QImage.Format_RGB888)
        qt_img_scaled = qimage.scaled(self.video_conf_frame.width(), self.video_conf_frame.height(), Qt.KeepAspectRatio)
        pixmap = QPixmap.fromImage(qt_img_scaled)
        self.video_conf_frame.setPixmap(pixmap)

    def closeEvent(self, event):
        if self.camera is not None:
            self.camera.stop()
            self.decode_thread.stop()
        if self.qr_client:
            self.qr_client.destroy_node()
            rp.shutdown()
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
            time.sleep(0.01)
    
    def stop(self):
        self.running = False
        self.cap.release()
        self.quit()

class MyCam(QThread):
    #시그널 종류 생성
    update = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.running = True
        
    def run(self):
        while self.running == True:

            self.update.emit()
            time.sleep(0.1)
    
    def stop(self):
        self.running = False

class RegisterCam(QThread):
    #시그널 종류 생성
    update = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.running = True
        
    def run(self):
        while self.running == True:

            self.update.emit()
            time.sleep(0.1)
    
    def stop(self):
        self.running = False

class CommuteCam(QThread):
    #시그널 종류 생성
    update = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.running = True
        
    def run(self):
        while self.running == True:

            self.update.emit()
            time.sleep(0.1)
    
    def stop(self):
        self.running = False

class QRDecodeThread(QThread):
    decoded_data_signal = pyqtSignal(str)
    last_frame_signal = pyqtSignal(np.ndarray)

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
            self.last_frame_signal.emit(frame)

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

    # image_subscriber = ImageSubscriber(signals)
    # myWindows.current_mode = "wooktest"
    # image_subscriber.mode = myWindows.current_mode
    # print("여기", image_subscriber.mode)
    face_name_subscriber = FacenameSubscriber(signals)
    face_details_subscriber = FacelandmarkSubscriber(signals)

    # myWindows.signals.update_mode_signal.connect(lambda mode: setattr(image_subscriber, 'mode', mode))

    # executor.add_node(image_subscriber)
    executor.add_node(face_name_subscriber)
    executor.add_node(face_details_subscriber)

    thread = Thread(target=executor.spin)
    thread.start()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()