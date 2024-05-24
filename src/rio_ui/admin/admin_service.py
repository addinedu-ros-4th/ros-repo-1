from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy 

from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from example_interfaces.msg import Float64MultiArray
import math

import os
import sys
import yaml

current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(current_dir, os.pardir, os.pardir))
sys.path.append(src_dir)

from rio_db_manager.db_manager import DBManager
from rio_db_manager.create_init_db import CreateInitDB

admin_ui = uic.loadUiType("../admin/admin_service.ui")[0]


class WindowClass(QMainWindow, admin_ui):
    def __init__(self, db_manager):
        super().__init__()
        
        self.setupUi(self)
        self.db_manager = db_manager
        self.select_all()
        self.detail_bt.clicked.connect(self.table_datail)
        self.requestButton1.clicked.connect(self.topic_test)
        
        self.feedback = None
        
        rclpy.init(args=None)
        self.nav = BasicNavigator()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.update_goal_pose()
        
        self.pose_node = rclpy.create_node("sub_pose")
        self.pose_subscription = self.pose_node.create_subscription(
            PoseWithCovarianceStamped, 
            "/amcl_pose", 
            self.pos_callback, 
            10
        )
        
        self.req_node = rclpy.create_node("req_node")
        self.req_subscription = self.req_node.create_subscription(
            Float64MultiArray,
            "/task_request",
            self.req_callback,
            10
        )
        
        self.path_node = rclpy.create_node("path_node")
        self.path_subscription = self.path_node.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin)
        self.timer.timeout.connect(self.updateMap)
        self.timer.start(100)
        
        self.xLine.setText("0")
        self.yLine.setText("0")
        self.yawLine.setText("0")
        
        my_map = "./rio_ui/admin/maps/map_name.yaml"
        with open(my_map) as f:
            map_data = yaml.full_load(f)
            
        self.map_resolution = map_data["resolution"]
        self.map_origin = map_data["origin"][:2]
            
        self.pixmap = QPixmap("./rio_ui/admin/maps/map_name.pgm")
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
        self.update_goal_pose()
        self.nav.goToPose(self.goal_pose)

    def pos_callback(self, data):
            
        self.x_location = data.pose.pose.position.x
        self.y_location = data.pose.pose.position.y

        
    def req_callback(self, msg):
        data = msg.data
        
        if len(data) == 3:
            self.xLine.setText(str(data[0]))
            self.yLine.setText(str(data[1]))
            self.yawLine.setText(str(data[2]))
            self.topic_test()
            
    
    def path_callback(self, msg):
        distance = self.calculate_total_distance(msg.poses)
        
        if distance < 0.4:
            distance = 0
        else:
            self.remainLine.setText(str("{:.2f}".format(distance)))
        
    def calculate_total_distance(self, poses):
        total_distance = 0.0
        for i in range(len(poses) - 1):
            total_distance += self.euclidean_distance(poses[i].pose.position, poses[i + 1].pose.position)
        return total_distance
    
    def euclidean_distance(self, p1, p2):
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
            
            
        
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
        rclpy.spin_once(self.pose_node, timeout_sec = 0.1)
        rclpy.spin_once(self.req_node, timeout_sec = 0.1)
        rclpy.spin_once(self.path_node, timeout_sec = 0.1)

    def closeEvent(self, event):
        self.timer.stop()
        rclpy.shutdown()
        event.accept()

    def select_all(self):
        tables = self.db_manager.show_tables()
        for table in tables:
            self.select_table_cbx.addItem(table)

    def table_datail(self):
        selected_table = self.select_table_cbx.currentText()
        detail_data = self.db_manager.read(selected_table)

        detail_table = self.findChild(QTableWidget, "detail_table")

        # 데이터가 없는 경우 테이블을 초기화하고 종료
        if not detail_data:
            detail_table.clear()
            return

        # 테이블의 열 수를 설정
        num_columns = len(detail_data[0].keys())
        detail_table.setColumnCount(num_columns)

        # 각 열에 대한 헤더 레이블 추가
        header_labels = list(detail_data[0].keys())
        detail_table.setHorizontalHeaderLabels(header_labels)

        # 데이터 표시
        detail_table.setRowCount(len(detail_data))
        for i, row_data in enumerate(detail_data):
            for j, column_name in enumerate(header_labels):
                item = QTableWidgetItem(str(row_data[column_name])) 
                detail_table.setItem(i, j, item)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    db_info = {
        'host': 'localhost',
        'port': 3306,  # MySQL 포트 번호
        'user': 'root',
        'password': '1234',
        'database': 'RiO_DB'
    }
    db_manager = DBManager(db_info)

    db_initializer = CreateInitDB(db_manager)
    db_initializer.create_database()
    db_initializer.create_tables()

    myWindow = WindowClass(db_manager)
    myWindow.show()
    sys.exit(app.exec_())