from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *

from ament_index_python.packages import get_package_share_directory
import rclpy

from example_interfaces.msg import Float64MultiArray
from rio_ui.admin_service import *
from rio_ui_msgs.srv import VisitInfo


import sys
import os
import json
from datetime import datetime

main_ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "user_service.ui")
sub_ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "pre_arrangement.ui")
user_ui = uic.loadUiType(main_ui_file)[0]
sub_ui = uic.loadUiType(sub_ui_file)[0]


class UserGUI(QMainWindow, user_ui):
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
        pre_arrangement_window = SubGUI()
        pre_arrangement_window.exec_()
    
    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()

class SubGUI(QDialog, sub_ui):
    def __init__(self):
        super().__init__()
        
        self.setupUi(self)

        rclpy.node.Node
        self.node = rclpy.create_node('subgui_node')
        self.cli = self.node.create_client(VisitInfo, 'generate_qr')  
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')              
        
        self.submit_bt.setDefault(True)
        self.submit_bt.clicked.connect(self.handle_submit)  

    def handle_submit(self):
        button_click_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        date = self.info_6.date().toString('yyyy-MM-dd')
        
        time = self.info_7.time().toString('hh:mm:ss')
        visit_datetime = date + " " + time
        visit_info = {
            "visit_place": self.info_1.currentText(),
            "purpose": self.info_2.currentText(),
            "name": self.info_3.text(),
            "phone_number": self.info_4.text(),
            "affiliation": self.info_5.text(),
            "visit_datetime": visit_datetime,
            "robot_guidance": self.info_8.currentText(),
            "status": "",
            "registered_at": button_click_time,
            "updated_at": ""
        }
        self.request_qr(visit_info)

    def request_qr(self, visit_info):
        request = VisitInfo.Request()
        request.visitor_info = json.dumps(visit_info)
        future = self.cli.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.node.get_logger().info(response)

        # future.add_done_callback(self.handle_response)
        # print("1")

    def handle_response(self, future):
        try:
            response = future.result()
            print("2")
            print(f"Response: success={response.success}, message={response.message}")
        except Exception as e:
            print("fail: {e}")



def main():
    app = QApplication(sys.argv)
    mainwindow = UserGUI()
    mainwindow.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()