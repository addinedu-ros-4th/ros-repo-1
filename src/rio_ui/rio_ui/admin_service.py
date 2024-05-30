# admin_service.py

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from example_interfaces.msg import Float64MultiArray
from threading import Thread
from PyQt5.QtCore import pyqtSignal, QObject
import math

import json
import qrcode
from rio_ui_msgs.srv import VisitInfo
from rio_db_manager.db_manager import DBManager
from rio_db_manager.create_init_db import CreateInitDB
from rio_crypto.data_encryptor import DataEncryptor

class UserService(Node):
    def __init__(self):
        super().__init__('admin_service')
        self.srv = self.create_service(VisitInfo, 'generate_qr', self.generate_qr_callback)      
        self.encryptor = DataEncryptor()

    def generate_qr_callback(self, request, response):
        print(request.visitor_info)
        visit_info = json.loads(request.visitor_info)
        # print(visit_info)
        encrypted_info = self.encryptor.encrypt_data(visit_info)
        qr = qrcode.make(encrypted_info)
        qr_code_path = './qr_code.png'
        qr.save(qr_code_path)
        response.qr_code_path = qr_code_path
        return response
    

class ROSNodeSignals(QObject):
    amcl_pose_received = pyqtSignal(float, float)
    path_distance_received = pyqtSignal(float)
    task_request_received = pyqtSignal(float, float, float)

class AmclSubscriber(Node):
    def __init__(self, signals):
        super().__init__("amcl_sub")
        self.signals = signals
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.map_callback,
            10)
        
    def map_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.signals.amcl_pose_received.emit(x, y)

class PathSubscriber(Node):
    def __init__(self, signals):
        super().__init__("path_sub")
        self.signals = signals
        self.subscription = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )
    
    def path_callback(self, msg):
        distance = self.calculate_total_distance(msg.poses)
        self.signals.path_distance_received.emit(distance)
            
    def calculate_total_distance(self, poses):
        total_distance = 0.0
        for i in range(len(poses) - 1):
            total_distance += self.euclidean_distance(poses[i].pose.position, poses[i + 1].pose.position)
        return total_distance
    
    def euclidean_distance(self, p1, p2):
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)

class RequestSubscriber(Node):
    def __init__(self, signals):
        super().__init__("req_sub")
        self.signals = signals
        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/task_request",
            self.req_callback,
            10
        )
        
    def req_callback(self, msg):
        data = msg.data
        if len(data) == 3:
            self.signals.task_request_received.emit(data[0], data[1], data[2])

class DBConnector():
    def __init__(self):
        super().__init__()
        self.db_manager = None
        self.db_connect()

    def db_connect(self):
        self.db_manager = DBManager("db_config.json")
        db_initializer = CreateInitDB(self.db_manager)
        db_initializer.create_database()
        db_initializer.create_tables()
        
        return self.db_manager
    
    def show_all_tables(self):
        tables = self.db_manager.show_tables()
        
        return tables

    def select_all(self, table):
        all_table_data = self.db_manager.read(table)

        return all_table_data
        
def main(args=None):
    rclpy.init(args=args)
    admin_service = UserService()
    rclpy.spin(admin_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
