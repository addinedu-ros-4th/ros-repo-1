import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from example_interfaces.msg import Float64MultiArray, Int64MultiArray

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QTableWidgetItem


import math

import os
import json
import base64 
import socket
import qrcode
import netifaces
import threading
import http.server
import socketserver
from datetime import datetime
from pyzbar.pyzbar import decode
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import hashes

from rio_ui_msgs.srv import VisitInfo
from rio_ui.key_save_load import KeySaveLoad
from rio_db_manager.db_manager import DBManager
from rio_db_manager.create_init_db import CreateInitDB

from twilio.rest import Client

class UserService(Node):
    def __init__(self):
        super().__init__('admin_service')
        self.srv = self.create_service(VisitInfo, 'generate_qr', self.generate_qr_callback)      
        self.cryptor = InfoCryptor()
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
        try:
            visit_info = json.loads(request.visitor_info)
            if not visit_info:
                raise ValueError("Empty visitor_info received")
            print(visit_info)

            encrypted_info = self.cryptor.encrypt_data(visit_info) # visit_info를 암호화

            qr = qrcode.QRCode(
                version=1,
                error_correction=qrcode.constants.ERROR_CORRECT_M,
                box_size=10,
                border=4,
            )
            qr.add_data(encrypted_info)
            qr.make(fit=True)
            qr_img = qr.make_image(fill='black', back_color='white')

            name = visit_info['name']
            qr_code_path = os.path.join(self.qr_code_dir, f"{name}_qr_code.png")
            qr_img.save(qr_code_path)

            visit_info['status'] = "not visited"
            visit_info['updated_at'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self.db_connector.db_manager.create("VisitorInfo", visit_info)
            print(f"QR code saved at: {qr_code_path}")

            phone_number = '+82' + visit_info['phone_number'][1:]
            port = self.find_free_port()
            ip_address = self.get_ip_address()
            qr_url = f'http://{ip_address}:{port}'

            server = ThreadedHTTPServer(ip_address, port, qr_code_path)
            server.start()
            self.get_logger().info(f"Server started at {qr_url} with QR {qr_code_path}")

            response.success = True
            response.message = f"QR code generated and server started at port {port}"
            response.qr_code_path = qr_code_path

            # sms service를 사용하려면 send_sms함수를 주석 해제
            self.send_sms(name, phone_number, qr_url)
            print(response)

        except json.JSONDecodeError:
            print("Failed to decode JSON from visitor info")
            response.success = False
            response.message = "Failed to decode JSON"
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            response.success = False
            response.message = "An unexpected error occurred"

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
    
# 데이터 암호화 및 복호화
class InfoCryptor():
    def __init__(self):
        key_save_load = KeySaveLoad()
        self.public_key = key_save_load.load_public_key() # 암호화 키
        self.private_key = key_save_load.load_private_key() # 복호화 키

    def encrypt_data(self, data):
        if isinstance(data, dict):
            data = json.dumps(data)

        encrypted_data = self.public_key.encrypt(
            data.encode('utf-8'),
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )
        encrypted_data = base64.b64encode(encrypted_data).decode('utf-8')

        return encrypted_data
    
    def decrypt_data(self, encrypted_data):
        encrypted_data = base64.b64decode(encrypted_data.encode('utf-8'))
        decrypted_data = self.private_key.decrypt(
            encrypted_data,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )
        decrypted_data = json.loads(decrypted_data.decode('utf-8'))

        return decrypted_data

    def decrypt_config(self, encrypted_file):
        with open(encrypted_file, 'r') as file:
            encrypted_data = file.read()
        decrypted_data = self.decrypt_data(encrypted_data)
        
        return decrypted_data

    def encrypt_config(self, config_file, output_file):
        with open(config_file, 'r') as file:
            config_data = json.load(file)
        encrypted_data = self.encrypt_data(config_data, self.public_key)
        with open(output_file, 'w') as file:
            file.write(encrypted_data)


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

class OrderSubscriber(Node):
    def __init__(self, ui):
        super().__init__("order_sub")
        self.ui = ui
        self.order_subs = self.create_subscription(
            Int64MultiArray,
            "/order_request",
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
            "/task_request",
            self.req_callback,
            10
        )
        
    def req_callback(self, msg):
        data = msg.data
        if len(data) == 3:
            self.signals.task_request_received.emit(data[0], data[1], data[2])


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
        

def main(args=None):
    rclpy.init(args=args)
    admin_service = UserService()
    rclpy.spin(admin_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
