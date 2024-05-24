from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *

import rclpy
from example_interfaces.msg import Float64MultiArray

import sys

user_ui = uic.loadUiType("./rio_ui/user/user_service.ui")[0]

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
        
        # QTimer 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(100)

    def publish_task(self):
        req = Float64MultiArray()
        req.data = [
            float(self.xLine.text()), 
            float(self.yLine.text()), 
            float(self.yawLine.text())
        ]
        self.publisher.publish(req)
        print("Published:", req.data)
    
    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)
    
    def closeEvent(self, event):
        self.timer.stop()
        rclpy.shutdown()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    sys.exit(app.exec_())