from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *

from ament_index_python.packages import get_package_share_directory
import rclpy

from example_interfaces.msg import Float64MultiArray

import sys
import os


ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "user_service.ui")
user_ui = uic.loadUiType(ui_file)[0]

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

    def publish_task(self):
        req = Float64MultiArray()
        req.data = [
            float(self.xLine.text()), 
            float(self.yLine.text()), 
            float(self.yawLine.text())
        ]
        self.publisher.publish(req)
        print("Published:", req.data)
    
    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()
        
def main():
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    sys.exit(app.exec_())
        
if __name__ == "__main__":
    main()