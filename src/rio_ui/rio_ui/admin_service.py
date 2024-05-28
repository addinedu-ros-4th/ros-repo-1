from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *

from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile, qos_profile_sensor_data


from nav2_simple_commander.robot_navigator import BasicNavigator

from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from example_interfaces.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

import math
import os
import sys
import yaml

from rio_db_manager.db_manager import DBManager
from rio_db_manager.create_init_db import CreateInitDB

ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "admin_service.ui")

admin_ui = uic.loadUiType(ui_file)[0]

class AmclSubscriber(Node):
    def __init__(self, ui):
        super().__init__("amcl_sub")
        self.ui = ui
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.map_callback,
            10)
        
        self.painter = QPainter(self.ui.Map_label.pixmap())
        self.painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))
        
        
    def map_callback(self, data):
        self.ui.x_location = data.pose.pose.position.x
        self.ui.y_location = data.pose.pose.position.y
        
    
class PathSubscriber(Node):
    def __init__(self, ui):
        super().__init__("path_sub")
        self.ui = ui
        self.subscription = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )
    
    def path_callback(self, msg):
        distance = self.calculate_total_distance(msg.poses)
        
        if distance < 0.4:
            distance = 0
        else:
            self.ui.remainLine.setText(str("{:.2f}".format(distance)))
            
    def calculate_total_distance(self, poses):
        total_distance = 0.0
        for i in range(len(poses) - 1):
            total_distance += self.euclidean_distance(poses[i].pose.position, poses[i + 1].pose.position)
        return total_distance
    
    def euclidean_distance(self, p1, p2):
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
    

class RequestSubscirber(Node):
    def __init__(self, ui):
        super().__init__("req_sub")
        self.ui = ui
        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/task_request",
            self.req_callback,
            10
        )
        
    def req_callback(self, msg):
        data = msg.data
        
        if len(data) == 3:
            self.ui.xLine.setText(str(data[0]))
            self.ui.yLine.setText(str(data[1]))
            self.ui.yawLine.setText(str(data[2]))
            self.ui.topic_test()
            
        
    

class WindowClass(QMainWindow, admin_ui):
    def __init__(self, db_manager=None):
        super().__init__()
        self.setupUi(self)
        self.requestButton1.clicked.connect(self.topic_test)
        
        self.nav = BasicNavigator()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.update_goal_pose()

        self.xLine.setText("0")
        self.yLine.setText("0")
        self.yawLine.setText("0")
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)
        
        self.db_manager = db_manager
        self.select_all()
        self.detail_bt.clicked.connect(self.table_detail)
        

        with open(os.path.join(get_package_share_directory("rio_main"), "maps", "map_name.yaml")) as f:
            map_data = yaml.full_load(f)

        self.map_resolution = map_data["resolution"]
        self.map_origin = map_data["origin"][:2]

        self.pixmap = QPixmap(os.path.join(get_package_share_directory("rio_main"), "maps", map_data["image"]))
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()
        self.image_scale = 2
        self.pixmap = self.pixmap.transformed(QTransform().scale(-1, -1))
        self.Map_label.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
        self.Map_label.setAlignment(Qt.AlignCenter)

        self.x_location = 0.0
        self.y_location = 0.0
        
    def select_all(self):
        tables = self.db_manager.show_tables()
        for table in tables:
            self.select_table_cbx.addItem(table)

    def table_detail(self):
        selected_table = self.select_table_cbx.currentText()
        detail_data = self.db_manager.read(selected_table)

        detail_table = self.findChild(QTableWidget, "detail_table")
        
         # 테이블의 열 수를 설정
        num_columns = len(detail_data[0])
        detail_table.setColumnCount(num_columns)

        # 각 열에 대한 헤더 레이블 추가
        header_labels = list(detail_data[0])
        detail_table.setHorizontalHeaderLabels(header_labels)

        # 데이터가 없는 경우 테이블을 초기화하고 종료
        if not detail_data:
            detail_table.clear()
            return

    
        # 데이터 표시
        detail_table.setRowCount(len(detail_data))
        for i, row_data in enumerate(detail_data):
            for j, column_name in enumerate(header_labels):
                item = QTableWidgetItem(str(row_data[column_name])) 
                detail_table.setItem(i, j, item)
        
        
        
    def update_goal_pose(self, x=None, y=None, yaw=None):
        if x is None:
            x = float(self.xLine.text())
        if y is None:
            y = float(self.yLine.text())
        if yaw is None:
            yaw = float(self.yawLine.text())

        roll, pitch, yaw = 0.0, 0.0, yaw
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]
        
    def topic_test(self):
        self.update_goal_pose()
        self.nav.goToPose(self.goal_pose)
        
    def update_map(self):
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
    
    def closeEvent(self, event):
        self.timer.stop()
        self.db_manager.close()
        rclpy.shutdown()
        event.accept()
        


def main():
    
    db_manager = DBManager("db_config.json")
    db_initializer = CreateInitDB(db_manager)
    db_initializer.create_database()
    db_initializer.create_tables()
    
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    app = QApplication(sys.argv)
    myWindow = WindowClass(db_manager)
    myWindow.show()
    
    amcl_subscriber = AmclSubscriber(myWindow)
    executor.add_node(amcl_subscriber)
    
    path_subscriber = PathSubscriber(myWindow)
    executor.add_node(path_subscriber)
    
    req_subscriber = RequestSubscirber(myWindow)
    executor.add_node(req_subscriber)
    
    thread = Thread(target = executor.spin)
    thread.start()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
