from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *

from ament_index_python.packages import get_package_share_directory
import rclpy

from example_interfaces.msg import Float64MultiArray

import sys
import os
import datetime

# current_dir = os.path.dirname(os.path.abspath(__file__))
# src_dir = os.path.abspath(os.path.join(current_dir, '../../..', 'rio_crypto'))
# sys.path.append(src_dir)

from rio_crypto.data_encryptor import DataEncryptor

main_ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "user_service.ui")
sub_ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "pre_arrangement.ui")
user_ui = uic.loadUiType(main_ui_file)[0]
sub_ui = uic.loadUiType(sub_ui_file)[0]

class WindowClass(QMainWindow, user_ui):
    def __init__(self):
        super().__init__()
        
        self.setupUi(self)
        
        # ROS 2 노드 초기화
        rclpy.init(args=None)
        self.node = rclpy.create_node("task_node")
        self.publisher = self.node.create_publisher(Float64MultiArray, "task_request", 10)
        
        # 버튼 클릭 시 publish_task 함수 호출
        self.callRobotButton.clicked.connect(self.publish_task)
        self.pre_arrangement_bt.clicked.connect(self.write_pre_arrangement)

    def publish_task(self):
        req = Float64MultiArray()
        req.data = [
            float(self.xLine.text()), 
            float(self.yLine.text()), 
            float(self.yawLine.text())
        ]
        self.publisher.publish(req)
        print("Published:", req.data)

    def write_pre_arrangement(self):
        pre_arrangement_window = SubWindowClass()
        pre_arrangement_window.exec_()
    
    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()

class SubWindowClass(QDialog, sub_ui):
    def __init__(self):
        super().__init__()
        
        self.setupUi(self)
        
        self.submit_bt.setDefault(True)
        self.submit_bt.clicked.connect(self.get_info)  
        self.encryptor = DataEncryptor()

    def get_info(self):
        self.visit_place = self.info_1.currentText()
        self.purpose = self.info_2.currentText()
        self.name = self.info_3.text()
        self.phone_number = self.info_4.text()
        self.affiliation = self.info_5.text()
        self.visit_date = self.info_6.date()
        self.visit_date = self.visit_date.toString('yyyy-MM-dd')
        self.visit_time = self.info_7.time()
        self.visit_time = self.visit_time.toString('hh:mm:ss')
        self.robot_guidance = self.info_8.currentText()   
        button_click_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        visit_info = {
            "visit_place": self.visit_place,
            "purpose": self.purpose,
            "name": self.name,
            "phone_number": self.phone_number,
            "affiliation": self.affiliation,
            "visit_date": self.visit_date,
            "visit_time": self.visit_time,
            "robot_guidance": self.robot_guidance,
            "status": "registered",
            "created_at": button_click_time,
            "updated_at": button_click_time
        }

        # return_data = self.encryptor.encrypt_data(visit_info)
        # print(return_data)

        return self.encryptor.encrypt_data(visit_info)

        
def main():
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()

    sys.exit(app.exec_())
        
if __name__ == "__main__":
    main()