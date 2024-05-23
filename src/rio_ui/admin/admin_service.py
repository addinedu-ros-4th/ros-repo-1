from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node

from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import os
import sys
import numpy as np
import yaml

admin_ui = uic.loadUiType("./gui/admin_service.ui")[0]


class WindowClass(QMainWindow, admin_ui):
    def __init__(self):
        super().__init__()
        
        self.setupUi(self)
        self.requestButton1.clicked.connect(self.topic_test)
        
        rclpy.init(args=None)
        self.nav = BasicNavigator()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.update_goal_pose()
        
        self.pose_node = rclpy.create_node("sub_pose")
        self.pose_subscription = self.pose_node.create_subscription(
            PoseWithCovarianceStamped, 
            "/amcl_pose", 
            self.callback, 
            10
        )
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin)
        self.timer.timeout.connect(self.updateMap)
        self.timer.start(100)
        
        self.yLine.setText("0")
        self.yawLine.setText("0")
        
        my_map = "./gui/maps/map_name.yaml"
        with open(my_map) as f:
            map_data = yaml.full_load(f)
            
        self.map_resolution = map_data["resolution"]
        self.map_origin = map_data["origin"][:2]
            
        self.pixmap = QPixmap("./gui/maps/map_name.pgm")
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()
        self.image_scale = 2
        self.pixmap = self.pixmap.transformed(QTransform().scale(-1, -1))
        self.Map_label.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
        self.Map_label.setAlignment(Qt.AlignCenter)
        
        self.x_location = 0.0
        self.y_location = 0.0

    def update_goal_pose(self):
        x, y, z = float(self.xLine.text()), float(self.yLine.text()), 0.0
      
        # yaw는 radian 단위
        roll, pitch, yaw = 0.0, 0.0, float(self.yawLine.text()) 

        quaternion = quaternion_from_euler(roll, pitch, yaw)

        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]

    def topic_test(self):
        self.nav.goToPose(self.goal_pose)
        i = 0
        # while not self.nav.isTaskComplete():
        #     feedback = self.nav.getFeedback()
        #     if feedback and i % 5 == 0:
        #         print("Distance remaining: " + "{:.2f}".format(feedback.distance_remaining) + " meters.")
        #     i += 1
        #     rclpy.spin_once(self.pose_node, timeout_sec=0.1)
        
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Success!!")
        elif result == TaskResult.FAILED:
            print("Failed!")

    def callback(self, data):
        q = [0, 0, 0, 0]
        self.x_location = data.pose.pose.position.x
        self.y_location = data.pose.pose.position.y
        
        # q[0] = data.pose.pose.orientation.x
        # q[1] = data.pose.pose.orientation.y
        # q[2] = data.pose.pose.orientation.z
        # q[3] = data.pose.pose.orientation.w
        
        # self.zz.setText(str(euler_from_quaternion(q)[2]))
        
    def updateMap(self):
        self.Map_label.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
        self.Map_label.setAlignment(Qt.AlignCenter)
        
        painter = QPainter(self.Map_label.pixmap())
        
        painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))
        
        x, y = self.calc_coord(self.x_location, self.y_location)
        
        painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
        painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), '1')
        
        
        
    def calc_coord(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution
        pos_y = (y - self.map_origin[1]) / self.map_resolution
        
        return pos_x, pos_y
    
    def ros_spin(self):
        rclpy.spin_once(self.pose_node, timeout_sec=0.1)

    def closeEvent(self, event):
        self.timer.stop()
        rclpy.shutdown()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    sys.exit(app.exec_())