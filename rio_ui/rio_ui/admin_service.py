import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from example_interfaces.msg import Float64MultiArray, Int64MultiArray

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QTableWidgetItem

import math
import os
import json
import hashlib
import socket
import qrcode
import netifaces
import threading
import http.server
import socketserver
from datetime import datetime

from rio_ui_msgs.srv import GenerateVisitQR, QRCheck, VisitorAlert
from rio_db_manager.db_manager import DBManager
from rio_db_manager.create_init_db import CreateInitDB

from twilio.rest import Client

class GenerateQRServer(Node):
    def __init__(self):
        super().__init__('generate_qr_server')
        self.srv = self.create_service(GenerateVisitQR, 'generate_qr', self.generate_qr_callback)      
        self.qr_code_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../../src/rio_ui/rio_ui/data/'))
        os.makedirs(self.qr_code_dir, exist_ok=True)

        self.db_connector = DBConnector()
        self.server = None

    def get_ip_address(self):
        add = netifaces.ifaddresses('wlo1')
        server_ip = add[netifaces.AF_INET][0]['addr']

        return server_ip

    def find_free_port(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('', 0))

            return s.getsockname()[1]

    def generate_qr_callback(self, request, response):
        visitor_info = request.visitor_info
        if visitor_info:
            try:
                visit_info = json.loads(visitor_info)
                print(visit_info)
                data_to_hash = f"{visit_info['name']}{visit_info['phone_number']}{visit_info['visit_datetime']}"
                hashed_data = self.hash_data(data_to_hash)

                qr = qrcode.QRCode(
                    version=1,
                    error_correction=qrcode.constants.ERROR_CORRECT_M,
                    box_size=10,
                    border=4,
                )

                qr.add_data(hashed_data)
                qr.make(fit=True)
                qr_img = qr.make_image(fill='black', back_color='white')

                name = visit_info['name']
                qr_code_path = os.path.join(self.qr_code_dir, f"{name}_qr_code.png")
                qr_img.save(qr_code_path)

                phone_number = '+82' + visit_info['phone_number'][1:]
                port = self.find_free_port()
                ip_address = self.get_ip_address()
                qr_url = f'http://{ip_address}:{port}'

                server = ThreadedHTTPServer(ip_address, port, qr_code_path)
                server.start()
                self.get_logger().info(f"Server started at {qr_url} with QR {qr_code_path}")

                # sms service를 사용하려면 send_sms함수를 주석 해제
                self.send_sms(name, phone_number, qr_url)
                # print(response)

                visit_info['status'] = "not visited"
                visit_info['updated_at'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                visit_info['hashed_data'] = hashed_data
                self.db_connector.db_manager.create("VisitorInfo", visit_info)
                
                response.success = True
                response.message = f"등록하신 {name}님의 방문 예약 문자를 전송하였습니다."
                response.qr_code_path = qr_code_path
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
                response.success = False
                response.message = "방문 예약 등록을 실패하였습니다. 관리자 서비스를 통해 문의해주세요!"
        else:
            raise ValueError("Empty visitor_info received")

        return response
    
    def load_config(self, config_file):
        config_path = os.path.join(get_package_share_directory("rio_ui"), "data", config_file)
        
        with open(config_path, 'r') as file:
            config = json.load(file)
            config = config["sms_tocken"]

        return config

    def send_sms(self, client_name, client_number, qr_url):
        sms_tocken = self.load_config('sms_config.json')

        account_sid = sms_tocken['account_sid']
        auth_token = sms_tocken['auth_token']
        client = Client(account_sid, auth_token)

        message = client.messages.create(
            messaging_service_sid=sms_tocken['messaging_service_sid'], 
            body=f'{client_name}님, 방문 시 QR을 제시해주세요. {qr_url}', 
            to=client_number
        )

        return print("Send QR message to visitor")

    def hash_data(self, data):
        sha256 = hashlib.sha256()
        sha256.update(data.encode('utf-8'))
        
        return sha256.hexdigest()


class HTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, qr_path, *args, **kwargs):
        self.qr_path = qr_path
        print(qr_path)
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        if self.path == '/':
            self.send_html()
        elif self.path == '/qr_code_image':
            self.send_image()
        else:
            self.send_error(404, "File Not Found")

    
    def send_html(self):
        html_content = """
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>Display Image</title>
            <style>
                body {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    justify-content: flex-start;
                    height: 100vh;
                    margin: 0;
                }
                img {
                    max-width: 100%;
                    height: auto;
                }
                h1 {
                    margin: 20px 0;
                }
            </style>
        </head>
        <body>
            <h1>QR Code Image</h1>
            <img src="/qr_code_image" alt="QR Code">
        </body>
        </html>
        """
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(html_content.encode('utf-8'))

    def send_image(self):
        if os.path.exists(self.qr_path):
            self.send_response(200)
            self.send_header("Content-type", "image/png")
            self.end_headers()
            with open(self.qr_path, 'rb') as file:
                self.wfile.write(file.read())
        else:
            self.send_error(404, "File Not Found")

class ThreadedHTTPServer(object):
    def __init__(self, host, port, qr_path):
        self.qr_path = qr_path
        handler = lambda *args, **kwargs: HTTPRequestHandler(self.qr_path, *args, **kwargs)
        self.server = socketserver.TCPServer((host, port), handler)
    
    def start(self):
        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.daemon = True
        server_thread.start()
    
    def shutdown(self):
        self.server.shutdown()
        self.server.server_close()   


class ROSNodeSignals(QObject):
    amcl_pose_received = pyqtSignal(float, float)
    path_distance_received = pyqtSignal(float)
    task_request_received = pyqtSignal(float, float, float)
    visitor_alert_received = pyqtSignal(list)
    

class AmclSubscriber(Node):
    def __init__(self, signals):
        super().__init__("amcl_sub")
        self.signals = signals
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose_1",
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
            "/plan_1",
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

class OrderSubscriber(Node):
    def __init__(self, ui):
        super().__init__("order_sub")
        self.ui = ui
        self.order_subs = self.create_subscription(
            Int64MultiArray,
            "/order_request_1",
            self.order_callback,
            10
        )
    
    def order_callback(self, msg):
        office_num = msg.data[0]
        request_time = msg.data[1]
        order_list = msg.data[2:]
        
        time_str = str(request_time)
        formatted_time_str = f"{time_str[:2]}:{time_str[2:4]}:{time_str[4:]}"
        order_str = ""
        for i, order in enumerate(order_list):
            if order <= 0:
                pass
            else:
                if i == 0:
                    order_str = order_str + f"Americano : {order}, "
                elif i == 1:
                    order_str = order_str + f"Latte : {order}, "
                elif i == 2:
                    order_str = order_str + f"Coke : {order}, "
                elif i == 3:
                    order_str = order_str + f"Snack : {order}, "
                    
        row_position = self.ui.requestTable.rowCount()
        self.ui.requestTable.insertRow(row_position)
        self.ui.requestTable.setItem(row_position, 0, QTableWidgetItem(formatted_time_str))
        self.ui.requestTable.setItem(row_position, 1, QTableWidgetItem(str(office_num)))
        self.ui.requestTable.setItem(row_position, 2, QTableWidgetItem(order_str))
        

class RequestSubscriber(Node):
    def __init__(self, signals):
        super().__init__("req_sub")
        self.signals = signals
        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/task_request_1",
            self.req_callback,
            10
        )
        
    def req_callback(self, msg):
        data = msg.data
        if len(data) == 3:
            self.signals.task_request_received.emit(data[0], data[1], data[2])


class QRCheckServer(Node):
    # has_visited = pyqtSignal(dict)
    def __init__(self, signals):
        super().__init__('qr_check_server')
        self.signals = signals
        self.srv = self.create_service(QRCheck, 'qr_check', self.qr_code_callback)
        self.db_connector = DBConnector()

    def qr_code_callback(self, request, response):
        hashed_data = request.hashed_data
        if hashed_data:
            try:
                criteria = {"hashed_data": hashed_data}
                result = self.db_connector.db_manager.read("VisitorInfo", criteria)
                print("DB 조회 결과: ", result)
                if result:
                    data = {"updated_at": datetime.now()}
                    self.db_connector.db_manager.update("VisitorInfo", data, criteria)
                    visited_msg = [{'name': item['name'], 'affiliation': item['affiliation'], 'visit_place': item['visit_place']} for item in result]
                    print(visited_msg)
                    # self.visitalert.send_visit_alert_req(visited_msg)
                    self.signals.visitor_alert_received.emit(visited_msg)
                    response.success = True
                    response.message = "QR Code found: " + hashed_data
                else:
                    response.success = False
                    response.message = "QR Code not found"
                self.get_logger().info(f'Received QR Code: {hashed_data}')
            except Exception as e:
                response.success = False
                response.message = hashed_data
                response.message = f'Error: {str(e)}'
                self.get_logger().error(f'Error processing QR Code: {hashed_data}, Error: {str(e)}')

        return response

class VisitorService(Node):
    def __init__(self):
        super().__init__('visitor_alert_client')
        self.cli = self.create_client(VisitorAlert, 'get_visitor_info_1')
        # self.cli = self.create_client(VisitorAlert, 'get_visitor_info')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...') 
        self.request = VisitorAlert.Request()

    def send_visit_alert_req(self, visitor_info):
        self.request.name = visitor_info[0]['name']
        self.request.affiliation = visitor_info[0]['affiliation']
        self.request.visit_place = visitor_info[0]['visit_place']
        future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        self.handle_response(future)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"sent message to {self.request.visit_place} to {response.success}")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

class DBConnector: # 싱글톤 패턴으로 구현
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(DBConnector, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):  # 이 코드가 init이 여러 번 호출되지 않도록 합니다.
            self.db_manager = None
            self.db_connect()
            self.initialized = True

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
    
    def insert_value(self, table, value):
        insert_into_table = self.db_manager.create(table, value)
        
        return insert_into_table
    
class RFIDSubscriber(Node):
    def __init__(self, db_manager = None):
        super().__init__("rfid_sub")
        self.data = [0, 0, 0]
        self.subscription = self.create_subscription(
            Int64MultiArray,
            "/rfid_info_1",
            self.info_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Int64MultiArray,
            "/rfid_renew_1",
            10
        )
        
        self.db_manager = db_manager
        
    def info_callback(self, msg):
        self.rfid_uid = msg.data[0]
        self.change_total = msg.data[1]
        self.user_credit = msg.data[2]
        self.ID_check()
        
        
    def ID_check(self):
        uid_list = []
        id_list = []
        detail_data = self.db_manager.read("UserInfo")
        try:
            for data in detail_data:
                uid = data.get('rfid_UID')
                name = data.get('user_id')  # assuming 'user_name' is the key for names in the data
                if uid != 0:
                    uid_list.append(uid)
                    id_list.append(name)
                    
            uid_list = list(dict.fromkeys(uid_list))      
            id_list = list(dict.fromkeys(id_list))   
            if self.rfid_uid in uid_list:
                index = uid_list.index(self.rfid_uid)
                self.id = id_list[index]
                
                current_credit = self.calc_total(self.rfid_uid)
                self.total_credit = current_credit + self.change_total
                self.data[0] = 1
                self.data[1] = self.rfid_uid
                self.data[2] = current_credit
                
                
            else:
                self.data[0] = 0
                self.data[1] = self.rfid_uid
                self.data[2] = 0
            
            msg = Int64MultiArray(data = self.data)
            self.publisher.publish(msg)
            
            self.insert_into_DB()
                
        except Exception as e:
            self.get_logger().error(f'Error check UID: {e}')
            
    def calc_total(self, id):
        try:
            total_list = []
            criteria = {"rfid_uid": id}
            change_total_log = self.db_manager.read("Payment", criteria, "payment_id")
            
            if change_total_log:  # change_total_log가 비어 있지 않을 때
                for data in change_total_log:
                    total = data.get("total_credit")
                    total_list.append(total)
                current_credit = total_list[-1] if total_list else 0  # total_list가 비어 있지 않을 때
            else:  # change_total_log가 비어 있을 때
                current_credit = 0
                
            return current_credit
        except Exception as e:
            self.get_logger().error(f'Error calc_total: {e}')
            
    def insert_into_DB(self):
        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
        self.user_data = {
            "date": formatted_datetime,
            "user_id": self.id,
            "rfid_UID": self.rfid_uid,
            "change_info": "buy",
            "change_credit": self.change_total,
            "total_credit": self.user_credit + self.change_total
        }
    
        self.db_manager.create("Payment", self.user_data)
        

def main(args=None):
    rclpy.init(args=args)
    admin_service = GenerateQRServer()
    qr_service = QRCheckServer()
    executor = MultiThreadedExecutor()
    executor.add_node(admin_service)
    executor.add_node(qr_service)
    visitor_alert_thread = threading.Thread(target=rclpy.spin, args=(qr_service.visitalert,), daemon=True)
    visitor_alert_thread.start()

    try:
        thread = threading.Thread(target=executor.spin)
        thread.start()
        # executor.spin()
    finally:
        thread.join()
        visitor_alert_thread.join()
        qr_service.destroy_node()
        admin_service.destroy_node()
        qr_service.visitalert.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
