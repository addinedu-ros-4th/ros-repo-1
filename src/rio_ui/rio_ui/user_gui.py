from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from ament_index_python.packages import get_package_share_directory
import rclpy

from rio_ui.admin_service import *
from rio_ui_msgs.srv import GenerateVisitQR, VisitorAlert
from rio_ui_msgs.msg import RobotCall
from rclpy.executors import MultiThreadedExecutor

import sys
import os
import json
import threading
from datetime import datetime
from PIL import Image

main_ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "user_service.ui")
sub_ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "pre_arrangement.ui")
order_ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "order_service.ui")
user_ui = uic.loadUiType(main_ui_file)[0]
sub_ui = uic.loadUiType(sub_ui_file)[0]
order_ui = uic.loadUiType(order_ui_file)[0]


class UserGUI(QMainWindow, user_ui):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        if not rclpy.ok():
            rclpy.init(args=None)
        self.robot_call_node = rclpy.create_node('robot_call_user_node')
        self.robot_request_publisher = self.robot_call_node.create_publisher(RobotCall, 'robot_call_user', 10)
        self.service_node = rclpy.create_node('visitor_alert_server')
        self.service = self.service_node.create_service(VisitorAlert, 'get_visitor_info', self.alert_callback)
        self.start_executor_spin()
        self.tts = TTSAlertService()
        # self.tts.run_tts("user_greeting", "안녕하세요!")
        self.logInPW.installEventFilter(self)
        
        self.db_connector = DBConnector()
        
        self.callRobotGroup.hide()
        self.serviceGroup.hide()
        self.scheduleGroup.hide()
        self.label_4.hide()
        self.label_8.hide()
        self.video_confGroup.hide()
        
        header = self.scheduleTable.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        
        self.logInPW.setEchoMode(QLineEdit.Password)
        
        self.timer = self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)
        self.timer.start(60000)
        self.update_time()

        self.robotTypeCBX.currentIndexChanged.connect(self.set_destination_info)
        self.destination.currentIndexChanged.connect(self.set_receiver)
        self.logInBtn.clicked.connect(self.log_in)
        self.sendRobotRequestBtn.clicked.connect(self.publish_request)
        self.pre_arrangement_bt.clicked.connect(self.write_pre_arrangement)
        self.orderBtn.clicked.connect(self.order_menu)
        self.selectRobotBtn.clicked.connect(self.select_robot)
        self.backBtn.clicked.connect(self.cancel_select_robot)

        # self.userserver = UserVideoServer('192.168.0.10', 5600, self.video_conf_frame)
        # threading.Thread(target=self.userserver.receive_images, daemon=True).start()

        self.enter_videomeet_bt.clicked.connect(self.enter_video_conf)
        self.connect_meet_bt.clicked.connect(self.connect_meeting)
        self.disconnect_meet_bt.clicked.connect(self.disconnect_meeting)
        self.pixmap = QPixmap()
        self.tcpip_server = None
    
    # video_conf_frame
    def enter_video_conf(self):
        self.callRobotGroup.hide()
        self.serviceGroup.hide()
        self.scheduleGroup.hide()
        self.video_confGroup.show()
        self.mycam = MyCam()
        self.mycam.daemon = True
        self.image_updater = ImageUpdater()
        self.camera_start()
        self.mycam.update.connect(self.update_mycam)
        # client = GuideVideoClient('192.168.0.17', 5607)
        # client.send_frame()
    
    def connect_meeting(self):
        self.client_node = GuideTCPIPClientNode('192.168.0.48', 5608)
        self.client_node.image_received.connect(self.update_somebody_cam) 

    def disconnect_meeting(self):
        self.video_confGroup.hide()

    def camera_start(self):
        self.mycam.running = True
        self.mycam.start()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("cannot open camera")
            return
        if self.tcpip_server is None:
            self.tcpip_server = UserTCPIPServer(self.image_updater)
    
    def camera_stop(self):
        self.mycam.running = False
        self.cap.release()

    def update_mycam(self):
        ret, image = self.cap.read()
        if ret:
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w, c = rgb_image.shape
            qimage = QImage(rgb_image.data, w, h, w * c, QImage.Format_RGB888)
            qt_img_scaled = qimage.scaled(self.video_conf_frame_2.width(), self.video_conf_frame_2.height(), Qt.KeepAspectRatio)
            self.pixmap = self.pixmap.fromImage(qt_img_scaled)
            self.video_conf_frame_2.setPixmap(self.pixmap)
            
            self.image_updater.update_image(image)

    @pyqtSlot(np.ndarray)
    def update_somebody_cam(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, c = rgb_image.shape
        qimage = QImage(rgb_image.data, w, h, w * c, QImage.Format_RGB888)
        qt_img_scaled = qimage.scaled(self.video_conf_frame.width(), self.video_conf_frame.height(), Qt.KeepAspectRatio)
        pixmap = QPixmap.fromImage(qt_img_scaled)
        self.video_conf_frame.setPixmap(pixmap)

    def select_robot(self):
        self.destination.clear()
        self.callRobotGroup.show()
        self.selectRobotBtn.hide()
        self.enter_videomeet_bt.hide()
        table = "OfficeInfo"
        column = "office_number"
        criteria = "close_date"
        destination_list = ["meeting_room", "share_space", "share_printer", "share_cafe"]
        
        result = self.db_connector.select_specific_null(table, column, criteria)
        if self.robotTypeCBX.currentIndex() == "deliverybot":
            for value in result:
                    self.destination.addItem(str(value['office_number'])) 
                    
        else:
            for value in result:
                self.destination.addItem(str(value['office_number'])) 
                    
        print(result)
                
        for dt in destination_list:
            self.destination.addItem(dt)  
            
        self.destination.setCurrentText(str(self.office_number))
        
        self.set_destination_info()
        
        
        # self.destination.setCurrentText(str(self.office_number))
        
    def cancel_select_robot(self):
        self.callRobotGroup.hide()
        self.selectRobotBtn.show()
        self.enter_videomeet_bt.show()
        
    def set_receiver(self):
        self.receiver.clear()
        table = "UserInfo"
        print(self.destination.currentIndex())
        if self.destination.currentText() in ["501", "502", "503"]:
            column = "user_name, company"
            criteria = {"office": self.destination.currentText()}
        else:
            column = "user_name"
            criteria = None
            self.company.setText(self.id)
            
        result = self.db_connector.select_specific(table, column, criteria)
        
        for value in result:
            self.receiver.addItem(str(value['user_name'])) 
            if self.destination.currentText() in ["501", "502", "503"]:
                self.company.setText(str(value["company"]))
        
    def set_destination_info(self):
        robot_type = self.robotTypeCBX.currentText()
        self.items.setText("")
        if robot_type == "deliverybot":
            self.destination.setEnabled(True)
            self.items.setReadOnly(False)
            # self.destination.removeItem(str(self.office_number))
            # self.receiver.setEnabled(True)
        else:
            self.destination.setEnabled(False)
            self.items.setReadOnly(True)
            # self.receiver.setEnabled(False)
            self.destination.setCurrentText(str(self.office_number))
            
        
    def update_time(self):
        current_time = datetime.now()
        formatted_time = current_time.strftime("%H:%M")
        self.UITimer.setText(formatted_time)
        
    def eventFilter(self, source, event):
        if source is self.logInPW and event.type() == event.KeyPress:
            if event.key() == Qt.Key_Return or event.key() == Qt.Key_Enter:
                self.log_in() 
                return True  
        return super().eventFilter(source, event)
        
    def log_in(self):
        self.id = self.logInID.text()
        self.pw = self.logInPW.text()

        
        if self.id == "" or self.pw == "":
            self.log_in_blank_warning()
            
        else:
            table = "OfficeInfo"
            column = "office_number, close_date"
            criteria = {"company_name": self.id, "password": self.pw}
            
            result = self.db_connector.select_specific(table, column, criteria)
            
            for value in result:
                if value["close_date"] is None:
                    self.office_number = value["office_number"]
                    self.welcome_user()
                else:
                    self.log_in_fail()
                    
    def welcome_user(self):
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText(f"{self.id}님 환영합니다!")
        self.tts.run_create_tts(f"{self.id}_user_greeting", f"{self.id}님 환영합니다!")
        confirmation.setWindowTitle("welcome")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.buttonClicked.connect(self.set_service_display)
        confirmation.exec_()  
        
    def log_in_fail(self):
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText("ID, 비번을 확인하세요")
        confirmation.setWindowTitle("Log in Fail")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.exec_()  
                    
    def set_service_display(self):
        self.officeNumber.setText(str(self.office_number))
        self.callRobotGroup.hide()
        self.serviceGroup.show()
        self.scheduleGroup.show()
        self.label_4.show()
        self.label_8.show()
        self.logInGroup.hide()
        self.label_3.setText(f"{self.id}님 좋은 하루 보내세요!!")
            
            
    def log_in_blank_warning(self):
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText("ID, 비밀번호를 입력해주세요")
        confirmation.setWindowTitle("Log in Blank")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.exec_()  
            

    def publish_request(self):
        current_time = datetime.now().time()
        time_str = current_time.strftime('%H%M%S')
        if len(time_str) == 5:
            time_str = "0" + time_str
        time_int = int(time_str)
        
        robot_type = self.robotTypeCBX.currentText()
        destination = self.destination.currentText()
        receiver = self.receiver.currentText()
        items = self.items.text()
        
            
        req = RobotCall()
        req.office_number = self.office_number
        req.date = time_int
        req.robot_type = robot_type
        if robot_type == "deliverybot":
            req.robot_mode = "delivery"
        elif robot_type == "guidebot":
            req.robot_mode = "guide"
        elif robot_type == "cleanerbot":
            req.robot_mode = "clean"
        elif robot_type == "patrolblt":
            req.robot_mode = "patrol"
        req.destination = destination 
        req.receiver = receiver
        req.items = items
        self.robot_request_publisher.publish(req)
        print("Published:", req)

    def write_pre_arrangement(self):
        pre_arrangement_window = SubGUI()
        pre_arrangement_window.exec_()
    
    def order_menu(self):
        order_window = OrderGUI(self)
        order_window.exec_()

    def start_executor_spin(self):
        # self.alert_thread = threading.Thread(target=rclpy.spin, args=(self.service_node,), daemon=True)
        # self.alert_thread.start()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.robot_call_node)
        self.executor.add_node(self.service_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def alert_callback(self, request, response):
        name = request.name
        affiliation = request.affiliation

        robot_guidance = request.robot_guidance
        QMetaObject.invokeMethod(self, "visitor_alert_wrapper", Qt.QueuedConnection, Q_ARG(str, name), Q_ARG(str, affiliation), Q_ARG(bool, robot_guidance))
        
        response.success = True
        return response

    @pyqtSlot(str, str, bool)
    def visitor_alert_wrapper(self, name, affiliation, robot_guidance):
        self.visitor_alert(name, affiliation, robot_guidance)

    def visitor_alert(self, name, affiliation, robot_guidance):
        self.tts.run_create_tts(f"{affiliation}_{name}_greeting", f"{affiliation}의 {name}님이 방문하셨습니다.")
        if robot_guidance == True:
            self.service_node.get_logger().info(f"손님 도착 알림, {affiliation}의 {name}님이 방문하였습니다.\n로봇 길 안내를 시작합니다.")
            QMessageBox.information(self, f"손님 도착 알림", f"{affiliation}의 {name}님이 방문하였습니다.\n로봇 길 안내를 시작합니다.")
        else:
            self.service_node.get_logger().info(f"손님 도착 알림, {affiliation}의 {name}님이 방문하였습니다.")
            QMessageBox.information(self, f"손님 도착 알림", f"{affiliation}의 {name}님이 방문하였습니다.")

    
    def closeEvent(self, event):
        self.executor.shutdown()
        self.executor_thread.join()
        rclpy.shutdown()
        self.tts.stop_tts()
        event.accept()
        
class OrderGUI(QDialog, order_ui):
    def __init__(self, ui):
        super().__init__()
        self.setupUi(self)
        self.ui = ui
        
        self.office_num = self.ui.office_number
        self.total_price = 0
        self.coke_price = 2000
        self.ame_price = 1000
        self.lat_price = 1500
        self.snack_price = 2000
        self.order = {"americano": 0, "latte": 0, "coke": 0, "snack": 0}
        self.db_connector = self.ui.db_connector
        
        self.officeNumber.setText(str(self.office_num))
        
        self.node = rclpy.create_node("order_node")
        self.order_publisher = self.node.create_publisher(RobotCall, "robot_call_user", 10)
        
        self.addCokeBtn.clicked.connect(self.add_coke)
        self.subCokeBtn.clicked.connect(self.sub_coke)
        self.addAmericanoBtn.clicked.connect(self.add_americano)
        self.subAmericanoBtn.clicked.connect(self.sub_americano)
        self.addLatteBtn.clicked.connect(self.add_latte)
        self.subLatteBtn.clicked.connect(self.sub_latte)
        self.addSnackBtn.clicked.connect(self.add_snack)
        self.subSnackBtn.clicked.connect(self.sub_snack)
        self.cancelBtn.clicked.connect(self.remove_list)
        self.orderBtn.clicked.connect(self.show_confirmation_dialog)
    
    def add_americano(self):
        item = "americano"
        row, number = self.find_item(item)
        self.add_item(row, item, number, self.ame_price)    
        
    def sub_americano(self):
        item = "americano"
        row, number = self.find_item(item)
        self.sub_item(row, item, number, self.ame_price)
    
    def add_latte(self):
        item = "latte"
        row, number = self.find_item(item)
        self.add_item(row, item, number, self.lat_price)
        
    def sub_latte(self):
        item = "latte"
        row, number = self.find_item(item)
        self.sub_item(row, item, number, self.lat_price)
        
    def add_coke(self):
        item = "coke"
        row, number = self.find_item(item)
        self.add_item(row, item, number, self.coke_price)
        
    def sub_coke(self):
        item = "coke"
        row, number = self.find_item(item)
        self.sub_item(row, item, number, self.coke_price)
    
    def add_snack(self):
        item = "snack"
        row, number = self.find_item(item)
        self.add_item(row, item, number, self.snack_price)
        
    def sub_snack(self):
        item = "snack"
        row, number = self.find_item(item)
        self.sub_item(row, item, number, self.snack_price)
        
    def find_item(self,item):
        found_items = self.orderTable.findItems(item, Qt.MatchExactly)
        
        if len(found_items) != 0:
            row = found_items[0].row()
            number = self.orderTable.item(row, 2).text()
        else:
            row = None
            number = None
        return row, number
    
    def add_item(self, row, item, num, price):
        if row is not None and num is not None:
            self.orderTable.setItem(int(row), 2, QTableWidgetItem(str(int(num) + 1)))
        else:
            row_position = self.orderTable.rowCount()
            self.orderTable.insertRow(row_position)
            self.orderTable.setItem(row_position, 0, QTableWidgetItem(item))
            self.orderTable.setItem(row_position, 1, QTableWidgetItem(str(price)))
            self.orderTable.setItem(row_position, 2, QTableWidgetItem("1"))
        self.total_price += price
        self.totalPrice.setText(str(self.total_price))
        self.order[item] += 1
    
    def sub_item(self, row, item, num, price):
        if num == "1":
            self.orderTable.removeRow(row)
            self.total_price -= price
            self.totalPrice.setText(str(self.total_price))
        elif num:
            self.orderTable.setItem(int(row), 2, QTableWidgetItem(str(int(num) - 1)))
            self.total_price -= price
            self.totalPrice.setText(str(self.total_price))
        else:
            pass
        
        if num == "1" or num:
            if self.order[item] <= 0:
                pass
            else:
                self.order[item] -= 1
    
    def remove_list(self):
        self.orderTable.clearContents()
        self.total_price = 0
        self.totalPrice.setText(str(self.total_price))
        
        for key in self.order:
            self.order[key] = 0
    
    def show_confirmation_dialog(self):
        if int(self.totalPrice.text()) <= 0:
            confirmation = QMessageBox()
            confirmation.setIcon(QMessageBox.Question)
            confirmation.setText("주문할 상품을 선택해주세요.")
            confirmation.setWindowTitle("주문 확인")
            confirmation.setStandardButtons(QMessageBox.Ok)
            confirmation.exec_()
        else:
            confirmation = QMessageBox()
            confirmation.setIcon(QMessageBox.Question)
            confirmation.setText("상품을 주문하시겠습니까?")
            confirmation.setWindowTitle("주문 요청")
            confirmation.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            confirmation.buttonClicked.connect(self.order_confirm)
            confirmation.exec_()

    def order_confirm(self):
        order_string = ""
        current_time = datetime.now().time()
        time_str = current_time.strftime('%H%M%S')
        if len(time_str) == 5:
            time_str = "0" + time_str
        time_int = int(time_str)
        
        item_list = list(self.order.keys())
        order_list = list(self.order.values())
        for i in range(4):
            order_string += f"{item_list[i]} : {order_list[i]}, "
        
        msg = RobotCall()
        msg.office_number = self.office_num
        msg.date = time_int
        msg.robot_type = "deliverybot"
        msg.robot_mode = "order"
        msg.destination = str(self.ui.office_number)
        msg.receiver = self.set_order_receiver(self.office_num)
        msg.items = order_string
        
        self.order_publisher.publish(msg)
        print("Published:", msg)
        
        self.remove_list()
        
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText("주문이 완료되었습니다.")
        confirmation.setWindowTitle("주문 완료")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.buttonClicked.connect(self.order_complete)
        confirmation.exec_()
        
    def set_order_receiver(self, office_number):
        table = "UserInfo"
        column = "user_name"
        criteria = {"office": office_number}
        users = ""
                    
        result = self.db_connector.select_specific(table, column, criteria)
        for user in result:
            users += user["user_name"] + ","
            
        return users
            
    def order_complete(self, button):
        self.accept()
    
    def closeEvent(self, event):
        self.node.destroy_node()
        event.accept()


class SubGUI(QDialog, sub_ui):
    def __init__(self):
        super().__init__()
        self.setupUi(self)     

        self.node = rclpy.create_node('generate_qr_client')
        # self.cli = self.node.create_client(GenerateVisitQR, 'generate_qr_1')  
        self.cli = self.node.create_client(GenerateVisitQR, 'generate_qr') 
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')              
        self.request = GenerateVisitQR.Request()

        self.tts = TTSAlertService()

        self.submit_bt.setDefault(True)
        self.submit_bt.clicked.connect(self.handle_submit)
        self.cancel_bt.clicked.connect(self.close_dialog)

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
        self.request.visitor_info = json.dumps(visit_info)
        future = self.cli.call_async(self.request)   
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.handle_response(future)
        # future.add_done_callback(self.handle_response)
        # rclpy.spin_until_future_complete(self.node, future)
        # self.handle_response(future)

    def handle_response(self, future):
        try:
            response = future.result()
            self.node.get_logger().info(f"Response: success={response.success}, message={response.message}")
            # self.anounce_sms_success(response.success, response.message)
            QMetaObject.invokeMethod(self, "announce_sms_success", Qt.QueuedConnection, Q_ARG(bool, response.success), Q_ARG(str, response.message))
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {e}')
    
    @pyqtSlot(bool, str)
    def announce_sms_success(self, success, message):
        if success:
            self.tts.run_tts("message_send_success")
            QMessageBox.information(self, "발송 완료",message)
        else:
            self.tts.run_create_tts(f"message_send_fail", f"메시지 전송을 실패하였습니다. 관리자에게 문의해주세요!")
            QMessageBox.warning(self, "발송 오류", message)
    
    def close_dialog(self):
        self.close()

    def closeEvent(self, event):
        self.node.destroy_node()
        event.accept()

class ImageUpdater(QObject):
    image_updated = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()

    def update_image(self, image):
        # 이미지 업데이트 로직
        self.image_updated.emit(image)

class MyCam(QThread):
    #시그널 종류 생성
    update = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.running = True
        
    def run(self):
        while self.running == True:

            self.update.emit()
            time.sleep(0.1)
    
    def stop(self):
        self.running = False


class UserTCPIPServer(QObject):
    def __init__(self, image_updater):
        super().__init__()
        self.image_updater = image_updater
        self.image_updater.image_updated.connect(self.publish_frames)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('192.168.0.24', 5604))
        self.server_socket.listen(1)
        print("waiting connection")
        
        self.client_socket = None
        self.connection = None
        self.accept_thread = threading.Thread(target=self.accept_connections)
        self.accept_thread.start()

    def accept_connections(self):
        try:
            while True:
                self.client_socket, self.client_address = self.server_socket.accept()
                self.connection = self.client_socket.makefile('wb')
                
        except Exception as e:
            print(f"accept_connection :{e}")

    def publish_frames(self, frame):
        if self.connection:
            try:
                encoded, buffer = cv2.imencode('.jpg', frame)
                data = np.array(buffer)
                string_data = data.tobytes()
                self.connection.write(struct.pack('<L', len(string_data)))
                self.connection.write(string_data)
                self.connection.flush()
                    
            except Exception as e:
                self.connection.close()
                self.client_socket = None
                self.connection = None
                print(f"publish_frames{e}")
                
    def stop_server(self):
        if self.connection:
            self.connection.close()
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()
        print("Server is closed")


class GuideTCPIPClientNode(QObject):
    image_received = pyqtSignal(np.ndarray)

    def __init__(self, ip_address, port):
        super().__init__()
        self.client_socket = None
        self.connection = None
        self.connect_to_server(ip_address, port)
        self.receive_thread = threading.Thread(target=self.receive_video)
        self.receive_thread.start()

            
    def connect_to_server(self, ip_address, port):
        while True:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((ip_address, port))
                self.connection = self.client_socket.makefile('rb')
                print(f"Connected to server at {ip_address}:{port}")
                break
            except Exception as e:
                print(f"Failed to connect to server at {ip_address}:{port}: {e}")
                time.sleep(5)

    def receive_video(self):
        while True:
            try:
                while True:
                    packed_msg_size = self.connection.read(struct.calcsize('<L'))
                    
                    if not packed_msg_size:
                        break
                    msg_size = struct.unpack('<L', packed_msg_size)[0]
                    frame_data = self.connection.read(msg_size)
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    
                    if frame is not None and frame.size != 0:
                        self.image_received.emit(frame)  # 이미지 수신 시그널 방출
                    else:
                        print("Invalid frame received")
            except Exception as e:
                print(f"Error: {e}")


def main():
    app = QApplication(sys.argv)
    mainwindow = UserGUI()
    mainwindow.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()