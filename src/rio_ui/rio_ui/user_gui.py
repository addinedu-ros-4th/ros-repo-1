from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *

from ament_index_python.packages import get_package_share_directory
import rclpy

from example_interfaces.msg import Float64MultiArray, Int64MultiArray
from rio_ui.admin_service import *
from rio_ui_msgs.srv import GenerateVisitQR, VisitorAlert

import sys
import os
import json
import threading
from datetime import datetime

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

        self.node = rclpy.create_node('task_node')
        self.publisher = self.node.create_publisher(Float64MultiArray, 'task_request', 10)
        self.service_node = rclpy.create_node('visitor_alert_server')
        self.service = self.service_node.create_service(VisitorAlert, 'get_visitor_info', self.alert_callback)
        self.start_node_spin()
        # 버튼 클릭 시 publish_task 함수 호출
        self.callRobotButton.clicked.connect(self.publish_task)
        self.pre_arrangement_bt.clicked.connect(self.write_pre_arrangement)
        self.orderBtn.clicked.connect(self.order_menu)

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
    
    def order_menu(self):
        order_window = OrderGUI()
        order_window.exec_()

    def start_node_spin(self):
        self.alert_thread = threading.Thread(target=rclpy.spin, args=(self.service_node,), daemon=True)
        self.alert_thread.start()

    def alert_callback(self, request, response):
        name = request.name
        affiliation = request.affiliation
        # 나중에 domain bridge 한다면 아래 변수로
        # visit_place = request.visit_place 
        robot_guidance = request.robot_guidance

        QMetaObject.invokeMethod(self, "visitor_alert_wrapper", Qt.QueuedConnection, Q_ARG(str, name), Q_ARG(str, affiliation), Q_ARG(bool, robot_guidance))
        
        response.success = True
        return response

    @pyqtSlot(str, str, bool)
    def visitor_alert_wrapper(self, name, affiliation, robot_guidance):
        self.visitor_alert(name, affiliation, robot_guidance)

    def visitor_alert(self, name, affiliation, robot_guidance):
        if robot_guidance == True:
            QMessageBox.information(self, "손님 도착 알림", f"{affiliation}의 {name}님이 도착하였습니다.\n로봇 길 안내를 시작합니다.")
        else:
            QMessageBox.information(self, "손님 도착 알림", f"{affiliation}의 {name}님이 도착하였습니다.")

    
    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()
        
class OrderGUI(QDialog, order_ui):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self.office_num = 309
        self.total_price = 0
        self.coke_price = 2000
        self.ame_price = 1000
        self.lat_price = 1500
        self.snack_price = 2000
        self.order = {"americano": 0, "latte": 0, "coke": 0, "snack": 0}
        
        self.officeNumber.setText(str(self.office_num))
        
        self.node = rclpy.create_node("order_node")
        self.order_publisher = self.node.create_publisher(Int64MultiArray, "order_request", 10)
        
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
        self.orderTable.setRowCount(0)
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
            confirmation.setText("결제를 진행하시겠습니까?")
            confirmation.setWindowTitle("결제 확인")
            confirmation.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            confirmation.buttonClicked.connect(self.order_confirm)
            confirmation.exec_()

    def order_confirm(self):
        current_time = datetime.now().time()
        time_str = current_time.strftime('%H%M%S')
        time_int = int(time_str)
        order = [self.office_num, time_int]
        order_list = list(self.order.values())
        order.extend(order_list)
        msg = Int64MultiArray()
        msg.data = order
        self.order_publisher.publish(msg)
        
        self.remove_list()
        
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText("주문이 완료되었습니다.")
        confirmation.setWindowTitle("주문 완료")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.buttonClicked.connect(self.order_complete)
        confirmation.exec_()
            
    def order_complete(self, button):
        self.accept()
    
    def closeEvent(self, event):
        event.accept()


class SubGUI(QDialog, sub_ui):
    def __init__(self):
        super().__init__()
        self.setupUi(self)     

        self.node = rclpy.create_node('subgui_node')
        self.cli = self.node.create_client(GenerateVisitQR, 'generate_qr')  
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')              
        self.request = GenerateVisitQR.Request()

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
        future.add_done_callback(self.handle_response)
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.handle_response(future)
        # rclpy.spin_until_future_complete(self.node, future)
        # self.handle_response(future)

    def handle_response(self, future):
        try:
            response = future.result()
            self.node.get_logger().info(f"Response: success={response.success}, message={response.message}")
            # self.anounce_sms_success(response.success, response.message)
            QMetaObject.invokeMethod(self, "anounce_sms_success", Qt.QueuedConnection, Q_ARG(bool, response.success), Q_ARG(str, response.message))
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {e}')
    
    @pyqtSlot(bool, str)
    def anounce_sms_success(self, success, message):
        if success == True:
            QMessageBox.information(self, "발송 완료",message)
        else:
            QMessageBox.warning(self, "발송 오류", message)
    
    def close_dialog(self):
        self.close()

    def closeEvent(self, event):
        self.node.destroy_node()
        event.accept()

def main():
    app = QApplication(sys.argv)
    mainwindow = UserGUI()
    mainwindow.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()