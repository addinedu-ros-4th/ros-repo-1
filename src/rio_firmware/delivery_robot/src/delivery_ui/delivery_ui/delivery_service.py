import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtCore import pyqtSignal, QThread, Qt, QObject
from PyQt5.QtGui import QImage

from std_msgs.msg import String
from example_interfaces.msg import Int64MultiArray
from sensor_msgs.msg import Image

from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO

from threading import Thread
import threading

import time
from cv_bridge import CvBridge, CvBridgeError

import socket
import cv2
import numpy as np
import struct
# import queue

class ImageUpdater(QObject):
    image_updated = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()

    def update_image(self, image):
        # 이미지 업데이트 로직
        self.image_updated.emit(image)


class TCPIPServer(QObject):
    def __init__(self, image_updater):
        super().__init__()
        self.image_updater = image_updater
        self.image_updater.image_updated.connect(self.publish_frames)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', 5607))
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
        # is_register_mode = (self.ui.current_mode == "register")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.ui.is_on_camera:
                self.ui.update_image_signal.emit(cv_image)
        except CvBridgeError as e:
            print(f"image_callback : {e}")

    def update_names(self, names):
        self.names = names
        
class Camera(QThread):
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


class FacenameSubscriber(Node):
    def __init__(self, ui):
        super().__init__("face_name_sub")
        self.ui = ui
        self.names_sub = self.create_subscription(
            String,
            '/face_names_1', 
            self.names_callback, 
            10
        )

        
    def names_callback(self, data):
        try:
            names = data.data.split(',')
            self.ui.name = names[0]
            if self.ui.is_on_camera:
                if names[0] == "이지호" or names[0] == "이정욱" or names[0] == "유동규":
                    self.ui.camera_stop()
                    self.ui.faceLabel.setText(f"{names[0]}님 환영합니다!!")
                    time.sleep(1)
                    if self.ui.robot_task == "order":
                        self.ui.set_order()
                    elif self.ui.robot_task == "delivery":
                        self.ui.notice_success_rfid()
                        self.ui.servo.set_angle(90)
                elif names[0] == "Unknown" or names[0] =="":
                    self.ui.faceLabel.setText("얼굴 정면이 보이게 카메라를 바라봐주세요!!")
                        
        except CvBridgeError as e:
            print(f"names_callback : {e}")  
            
class TaskSubscriber(Node):
    def __init__(self, ui):
        super().__init__("task_sub")
        self.ui = ui
        self.subscription = self.create_subscription(
            Int64MultiArray,
            "/robot_task_1",
            self.order_callback,
            10)

    def order_callback(self, msg):
        try:
            if msg.data[1] == 1:
                self.ui.robot_task = "delivery"
            elif msg.data[1] == 2:
                self.ui.robot_task = "order"
            elif msg.data[1] == 3:
                self.ui.robot_task = "sale"
            else:
                self.ui.robot_task = "ready"
            self.order_list = [0, 0, 0, 0]
            self.order_list[0] = msg.data[2] # americano
            self.order_list[1] = msg.data[3] # latte
            self.order_list[2] = msg.data[4] # coke
            self.order_list[3] = msg.data[5] # snack

            if self.ui.robot_task == "delivery":
                self.ui.notice_success_rfid()
            elif self.ui.robot_task == "order":
                self.make_order_list()
                self.ui.is_on_camera = False
           
            
        except Exception as e:
            self.get_logger().error(f'Error reading RFID: {e}')
            
    def make_order_list(self):
        total_price = 0
        for item_num, order in enumerate(self.order_list):
            if item_num == 0:
                price = self.ui.ame_price
                item = self.ui.label_4.text()
            elif item_num == 1:
                price = self.ui.lat_price
                item = self.ui.label_7.text()
            elif item_num == 2:
                price = self.ui.coke_price
                item = self.ui.label_6.text()
            elif item_num == 3:
                price = self.ui.snack_price
                item = self.ui.label_5.text()
                
            i = 0
            if order != 0:
                self.ui.orderTable.insertRow(i)
                self.ui.orderTable.setItem(i, 0, QTableWidgetItem(item))
                self.ui.orderTable.setItem(i, 1, QTableWidgetItem(str(price)))
                self.ui.orderTable.setItem(i, 2, QTableWidgetItem(str(order)))
                i += 1
                total_price += price * order
                
        self.ui.total_price = total_price
        self.ui.totalPrice.setText(str(total_price)) 
        self.ui.is_got_order = False           

class RFIDReaderNode(Node):
    def __init__(self, ui):
        super().__init__('rfid_reader')
        
        GPIO.setwarnings(False)
        self.ui = ui

        self.reader = SimpleMFRC522()
        self.get_logger().info('RFID Reader Node has been started.')
        self.publisher = self.create_publisher(Int64MultiArray, "/rfid_info", 10)
        self.id = None
        self.is_on_rfid = False
        self.total_price = 0
        self.is_first_tag = True

        # ROS2 타이머를 사용하여 주기적으로 RFID 읽기
        self.timer_period = 1.0  # 1초마다 실행
        self.timer = self.create_timer(self.timer_period, self.read_rfid)

    def read_rfid(self):
        try:
            self.is_on_rfid = self.ui.is_on_rfid
            id = self.reader.read_id()
            
            if not self.ui.is_tag:  
                self.id = None  
                self.ui.is_tag = True
        
            if self.is_on_rfid:
                id, current_credit = self.reader.read()
                self.get_logger().info(f"RFID ID: {id}, Text: {current_credit}")
                cleaned_string = current_credit.replace('\0', '')
                
                if cleaned_string == "":
                    current_credit = 0
                else: 
                    current_credit = int(current_credit)
                
                if id != self.id:
                    msg = Int64MultiArray()
                    msg.data = [int(id), self.total_price, current_credit]
                    self.publisher.publish(msg)
                    self.id = id
                
                
                
        except Exception as e:
            self.get_logger().error(f'Error reading RFID: {e}')
            self.is_on_rfid = False

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

class RFIDSubscriber(Node):
    def __init__(self, rfid_node, ui):
        super().__init__("rfid_sub")
        self.rfid_node = rfid_node
        self.ui = ui
        
        self.subscription = self.create_subscription(
            Int64MultiArray,
            "/rfid_renew_1",
            self.rfid_callback,
            10)
        self.check_id = False
        self.servo_controller = ServoController()
        self.servo_controller.set_angle(0)

    def rfid_callback(self, msg):
        try:
            if msg.data[0] == 1:
                self.check_id = True
                
            uid = msg.data[1]
            
            id = self.rfid_node.reader.read_id()
            if id == uid and self.check_id:
                time.sleep(1)
                self.servo_controller.set_angle(90)
                self.ui.notice_success_rfid()
            elif not self.check_id:
                self.ui.label_8.setText("Unregistered Card")
            else:
                self.ui.label_8.setText("Try Again!!")
                self.ui.is_tag = False
            
        except Exception as e:
            self.get_logger().error(f'Error reading RFID: {e}')
            

class ServoController:
    pwm_instances = {}  # 클래스 변수를 사용하여 PWM 객체를 추적

    def __init__(self, servo_pin=17, pwm_freq=50):
        self.servo_pin = servo_pin
        self.PWM_FREQ = pwm_freq

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        try:
            # 이미 해당 채널에 대한 PWM 객체가 있는지 확인
            if self.servo_pin in ServoController.pwm_instances:
                self.pwm = ServoController.pwm_instances[self.servo_pin]
                print("PWM object already exists for this GPIO channel")
            else:
                self.pwm = GPIO.PWM(self.servo_pin, self.PWM_FREQ)
                self.pwm.start(0)
                ServoController.pwm_instances[self.servo_pin] = self.pwm  # PWM 객체를 클래스 변수에 저장
        except ValueError as e:
            print(f"Error in setting up GPIO pin: {e}")
            GPIO.cleanup()

    def set_angle(self, angle):
        try:
            duty = angle / 18.0 + 2.5
            self.pwm.ChangeDutyCycle(duty)
            time.sleep(1)  # 서보가 목표 각도로 이동할 시간을 줍니다.
            self.pwm.ChangeDutyCycle(0)  # 서보를 비활성화하여 과열을 방지합니다.
        except Exception as e:
            print(f"set_angle : {e}")


    def cleanup(self):
        self.pwm.stop()


def main(args=None):
    rclpy.init()

    executor = MultiThreadedExecutor()

    rfid_reader = RFIDReaderNode()
    rfid_subs = RFIDSubscriber(rfid_reader)

    executor.add_node(rfid_reader)
    executor.add_node(rfid_subs)

    thread = Thread(target=executor.spin)
    thread.start()

    try:
        rclpy.spin_until_future_complete(rfid_reader.get_logger().get_clock().create_future())
    finally:
        thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

