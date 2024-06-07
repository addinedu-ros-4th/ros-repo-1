from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import uic

from threading import Thread

import rclpy as rp

from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory


import numpy as np
import os
import sys
import cv2
import resource_rc

from delivery_service import *

ui_file = os.path.join(get_package_share_directory("delivery_ui"), "ui", "delivery_service.ui")

delivery_ui = uic.loadUiType(ui_file)[0]

class WindowClass(QMainWindow, delivery_ui):

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('RiO Deli mk3')

        self.robot_task = "delivery" # 초기 모드 설정
        self.robot_status = "ready"
        self.name = "Unknown"
        self.item = ""
        self.total_price = 0
        self.is_on_rfid = False
        self.is_tag = False
        self.is_on_camera = False
        self.is_got_order = True
        self.ame_price = 1000
        self.lat_price = 1500
        self.coke_price = 2000
        self.snack_price = 2000
        self.cap = None
        self.tcpip_server = None
    
        self.servo = ServoController()
        self.camera = Camera()
        self.camera.daemon = True
        self.image_updater = ImageUpdater()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_out)
 
        self.pixmap = QPixmap()
        self.pixmap = self.pixmap.scaled(self.label.width(), self.label.height())
        self.orderTable.verticalHeader().setVisible(False)

        self.camera.update.connect(self.update_camera)
        self.set_normal_display()
        
        self.normalBtn.clicked.connect(self.set_display)
        self.payBtn.clicked.connect(self.show_confirmation_dialog)
        self.backBtn.clicked.connect(self.set_normal_display)
        self.addCokeBtn.clicked.connect(self.add_coke)
        self.subCokeBtn.clicked.connect(self.sub_coke)
        self.addAmericanoBtn.clicked.connect(self.add_americano)
        self.subAmericanoBtn.clicked.connect(self.sub_americano)
        self.addLatteBtn.clicked.connect(self.add_latte)
        self.subLatteBtn.clicked.connect(self.sub_latte)
        self.addSnackBtn.clicked.connect(self.add_snack)
        self.subSnackBtn.clicked.connect(self.sub_snack)
        self.cancelBtn.clicked.connect(self.remove_list)
        self.closeBtn.clicked.connect(self.close_case)
        
    def set_normal_display(self):
        self.normalBtn.show()
        self.cameraGroup.hide()
        self.orderGroup.hide()
        self.menuGroup.hide()
        self.rfidGroup.hide()
        self.noticeGroup.hide()
        self.totalPrice.setText("0")
        self.faceLabel.setText("얼굴 정면이 보이게 카메라를 바라봐주세요!!")
        self.label_8.setText("Tag your Card!!")
        self.name = "Unknown"
        self.total_price = 0
        self.is_on_rfid = False
        self.is_on_camera = False
        self.is_got_order = False
        self.servo.set_angle(0)
        try:
            self.camera_stop()
        except AttributeError as e:
            pass
       
    def set_camera(self):
        self.normalBtn.hide()
        self.cameraGroup.show()
        self.orderGroup.hide()
        self.menuGroup.hide()
        self.rfidGroup.hide()
        self.noticeGroup.hide()
        self.is_on_camera = True
        self.camera_start()
        self.timer.setInterval(20000)


    def set_order(self):
        self.normalBtn.hide()
        self.cameraGroup.hide()
        self.orderGroup.show()
        self.menuGroup.hide()
        self.rfidGroup.hide()
        self.noticeGroup.hide()
        self.is_on_camera = False
        self.is_got_order = True
        try:
            self.camera_stop()
        except AttributeError as e:
            pass
        self.timer.setInterval(20000)

    def set_sale(self):
        self.normalBtn.hide()
        self.cameraGroup.hide()
        self.orderGroup.show()
        self.menuGroup.show()
        self.rfidGroup.hide()
        self.noticeGroup.hide()
        self.is_on_camera = False
        self.timer.setInterval(20000)
        
    def set_rfid(self):
        self.normalBtn.hide()
        self.cameraGroup.hide()
        self.orderGroup.hide()
        self.menuGroup.hide()
        self.rfidGroup.show()
        self.noticeGroup.hide()
        self.is_on_rfid = True
        self.timer.setInterval(20000)
        
    def notice_success_rfid(self):
        self.normalBtn.hide()
        self.cameraGroup.hide()
        self.orderGroup.hide()
        self.menuGroup.hide()
        self.rfidGroup.hide()
        self.noticeGroup.show()
        self.is_on_camera = False
        self.is_tag = False
        if self.robot_task == "delivery":
            self.label_11.hide()
        elif self.robot_task == "order" or self.robot_task == "sale":
            self.label_11.show()
            
        try:
            self.camera_stop()
        except AttributeError as e:
            pass
        
        self.timer.setInterval(20000)
             
    def set_display(self):
        self.timer.start(20000)
        if self.robot_status == "ready":
            if self.robot_task == "sale":
                self.set_sale()
                
            elif self.robot_task == "delivery" or self.robot_task == "order":
                self.set_camera()
            else:
                self.set_normal_display()
                
    def timer_out(self):
        self.servo.set_angle(0)
        self.set_normal_display()
        self.timer.stop()
        if self.robot_task == "sale":
            self.remove_list()
        elif self.robot_task == "order":
            pass

    
    def camera_start(self):
        self.camera.running = True
        self.camera.start()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("cannot open camera")
            return
        if self.tcpip_server is None:
            self.tcpip_server = TCPIPServer(self.image_updater)
        
        
    def update_camera(self):
        ret, image = self.cap.read()
        if ret:
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w, c = rgb_image.shape
            qimage = QImage(rgb_image.data, w, h, w * c, QImage.Format_RGB888)
            qt_img_scaled = qimage.scaled(self.label.width(), self.label.height(), Qt.KeepAspectRatio)
            self.pixmap = self.pixmap.fromImage(qt_img_scaled)
            self.label.setPixmap(self.pixmap)
            
            self.image_updater.update_image(image)
            
    def camera_stop(self):
        self.camera.running = False
        self.cap.release()
        # self.tcpip_server.stop_server()
    
    
    def add_coke(self):
        item = "coke"
        row, number = self.find_item(item)
        self.add_item(row, item, number, self.coke_price)
        
            
    def sub_coke(self):
        item = "coke"
        row, number = self.find_item(item)
        self.sub_item(row, number, self.coke_price)
    
    def add_americano(self):
        item = "americano"
        row, number = self.find_item(item)
        self.add_item(row, item, number, self.ame_price)
        
    def sub_americano(self):
        item = "americano"
        row, number = self.find_item(item)
        self.sub_item(row, number, self.ame_price)
    
    def add_latte(self):
        item = "latte"
        row, number = self.find_item(item)
        self.add_item(row, item, number, self.lat_price)
        
    def sub_latte(self):
        item = "latte"
        row, number = self.find_item(item)
        self.sub_item(row, number, self.lat_price)
    
    def add_snack(self):
        item = "snack"
        row, number = self.find_item(item)
        self.add_item(row, item, number, self.snack_price)
        
    def sub_snack(self):
        item = "snack"
        row, number = self.find_item(item)
        self.sub_item(row, number, self.snack_price)
        
    def find_item(self,item):
        found_items = self.orderTable.findItems(item, Qt.MatchExactly)
        
        if len(found_items) != 0:
            row = found_items[0].row()
            number = self.orderTable.item(row, 2).text()
        else:
            row = None
            number = None
        return row, number
    
    def add_item(self, row, item, num, price):
        if row is not None and num is not None:
            self.orderTable.setItem(int(row), 2, QTableWidgetItem(str(int(num) + 1)))
        else:
            row_position = self.orderTable.rowCount()
            self.orderTable.insertRow(row_position)
            self.orderTable.setItem(row_position, 0, QTableWidgetItem(item))
            self.orderTable.setItem(row_position, 1, QTableWidgetItem(str(price)))
            self.orderTable.setItem(row_position, 2, QTableWidgetItem("1"))
        self.total_price += price
        self.totalPrice.setText(str(self.total_price))
    
    def sub_item(self, row, num, price):
        if num == "1":
            self.orderTable.removeRow(row)
            self.total_price -= price
            self.totalPrice.setText(str(self.total_price))
        elif num:
            self.orderTable.setItem(int(row), 2, QTableWidgetItem(str(int(num) - 1)))
            self.total_price -= price
            self.totalPrice.setText(str(self.total_price))
        else:
            pass
    
    def remove_list(self):
        self.orderTable.setRowCount(0)
        self.total_price = 0
        self.totalPrice.setText(str(self.total_price))
    
    def show_confirmation_dialog(self):
        if int(self.totalPrice.text()) <= 0:
            confirmation = QMessageBox()
            confirmation.setIcon(QMessageBox.Question)
            confirmation.setText("주문할 상품을 선택해주세요.")
            confirmation.setWindowTitle("주문 확인")
            confirmation.setStandardButtons(QMessageBox.Ok)
            confirmation.buttonClicked.connect(self.set_sale)
            confirmation.exec_()
        else:
            confirmation = QMessageBox()
            confirmation.setIcon(QMessageBox.Question)
            confirmation.setText("결제를 진행하시겠습니까?")
            confirmation.setWindowTitle("결제 확인")
            confirmation.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            confirmation.buttonClicked.connect(self.confirmation_button_clicked)
            confirmation.exec_()

    def confirmation_button_clicked(self, button):
        if button.text() == "OK":
            self.set_rfid()             
            

    def close_case(self):
        self.servo.set_angle(0)
        self.set_normal_display()
            
            


def main():
    rp.init()
    executors = MultiThreadedExecutor()
    

    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    

    # tcpip_server = TCPIPServer()
    # executors.add_node(tcpip_server)
    
    rfid_reader = RFIDReaderNode(myWindows)     
    executors.add_node(rfid_reader)
    
    rfid_subs = RFIDSubscriber(rfid_reader, myWindows)
    executors.add_node(rfid_subs)

    # image_subscriber = ImageSubscriber(myWindows)
    # executors.add_node(image_subscriber)
    
    name_subscriber = FacenameSubscriber(myWindows)
    executors.add_node(name_subscriber)
    
    task_subscriber = TaskSubscriber(myWindows)
    executors.add_node(task_subscriber)
    
    thread = Thread(target=executors.spin)
    thread.start()
    
    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()
        
    

if __name__ == "__main__":
    main()
    