from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *
import sys
import os
import yaml
import re
import glob
from io import BytesIO
from PIL import Image

from example_interfaces.msg import Int64MultiArray
from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from rio_ui.admin_service import *

admin_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "admin_service.ui")
add_user_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "add_user.ui")
admin_ui = uic.loadUiType(admin_file)[0]
add_user_ui = uic.loadUiType(add_user_file)[0]


class AdminGUI(QMainWindow, admin_ui):
    def __init__(self):    
        super().__init__()
        self.setupUi(self)
        self.requestButton1.clicked.connect(self.topic_test)
        self.deliRequestBtn.clicked.connect(self.pub_task)
        self.addUserBtn.clicked.connect(self.add_user)
        
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
        
        self.order = []
        self.node = rclpy.create_node("robot_task_node")
        self.task_publisher = self.node.create_publisher(Int64MultiArray, "/robot_task_1", 10)

        self.db_connector = DBConnector()
        # self.db_manager = db_manager
        tables = self.db_connector.show_all_tables()
        self.select_table_update(tables)
        self.detail_bt.clicked.connect(self.table_detail)
        
        self.signals = ROSNodeSignals()
        self.signals.amcl_pose_received.connect(self.update_amcl_pose)
        self.signals.path_distance_received.connect(self.update_path_distance)
        self.signals.task_request_received.connect(self.update_task_request)
        self.signals.visitor_alert_received.connect(self.visitor_alert_to_user)

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
        
        header = self.requestTable.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)

        self.x_location = 0.0
        self.y_location = 0.0

    def visitor_alert_to_user(self, message):
        visitor_alert = VisitorService()
        visitor_alert.send_visit_alert_req(message)
        name = message[0]['name']
        affiliation = message[0]['affiliation']
        # visit_place = message[0]['visit_place']
        self.visited_label.setText(f'{affiliation}소속의 {name}님이 방문하였습니다.')

        self.visit_timer = QTimer(self)
        self.visit_timer.setSingleShot(True)
        self.visit_timer.timeout.connect(self.clear_visited_label)
        self.visit_timer.start(3000)

    def clear_visited_label(self):
        self.visited_label.clear()

    def update_amcl_pose(self, x, y):
        self.x_location = x
        self.y_location = y

    def update_path_distance(self, distance):
        if distance < 0.4:
            distance = 0
        self.remainLine.setText(str("{:.2f}".format(distance)))

    def update_task_request(self, x, y, yaw):
        self.xLine.setText(str(x))
        self.yLine.setText(str(y))
        self.yawLine.setText(str(yaw))
        self.topic_test()

    def select_table_update(self, tables):
        self.select_table_cbx.clear()
        for table in tables:
            self.select_table_cbx.addItem(table)

    def table_detail(self):
        selected_table = self.select_table_cbx.currentText()
        detail_data = self.db_connector.select_all(selected_table)

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
    
    def pub_task(self):
        order = [0, 0, 0, 0, 0, 0]
        table_element = []
        pattern = r'\d+'
        column_count = self.requestTable.columnCount()
        row_count = self.requestTable.rowCount()
        
        if row_count > 0:
            for column in range(column_count):
                item = self.requestTable.item(0, column)  # 0 is the index of the first row
                if item is not None:
                    table_element.append(item.text())
            
            self.requestTable.removeRow(0)
            
            order_str = table_element[2].split(",")
            
            for item in order_str:
                numbers = [int(num) for num in re.findall(pattern, item)]
                
                if "Americano" in item:
                    order[2] = numbers[0] if numbers else 0
                elif "Latte" in item:
                    order[3] = numbers[0] if numbers else 0
                elif "Coke" in item:
                    order[4] = numbers[0] if numbers else 0
                elif "Snack" in item:
                    order[5] = numbers[0] if numbers else 0

            order[0] = int(table_element[1])
            order[1] = 2
            
            msg = Int64MultiArray()
            msg.data = order
            self.task_publisher.publish(msg)
            self.node.get_logger().info('Published message to /robot_task_1: %s' % order)
            
    def add_user(self):
        pre_arrangement_window = AddUserGUI()
        pre_arrangement_window.exec_()
                
    def closeEvent(self, event):
        self.timer.stop()
        if self.db_connector and self.db_connector.db_manager:
            self.db_connector.db_manager.close()
        rclpy.shutdown()
        event.accept()
        
class AddUserGUI(QDialog, add_user_ui):
    def __init__(self):
        super().__init__()
        
        self.setupUi(self)
        self.db_connector = DBConnector()
        
        self.img_data = ""
        
        reg_ex = QRegExp("[0-9]+")
        input_validator = QRegExpValidator(reg_ex, self.phoneInfo)
        
        self.phoneInfo.setValidator(input_validator)
        
        self.birthInfo.setDate(QDate.currentDate())
        self.birthInfo.setCalendarPopup(True)
        self.birthInfo.setDisplayFormat("yyyy-MM-dd")
        
        self.addBtn.clicked.connect(self.read_info)
        self.cancel_bt.clicked.connect(self.close)
        self.findImageBtn.clicked.connect(self.find_image)
        
    def find_image(self):
        self.file, _ = QFileDialog.getOpenFileName(self,"파일 선택", "", "Images (*.png *.jpg *.bmp)")
        self.faceInfo.setText(self.file)
        buffer = BytesIO()
        face_img = Image.open(self.file)
        face_img.save(buffer, "JPEG")
        self.img_data = buffer.getvalue()
        buffer.close()
        
    def read_info(self):
        name = self.nameInfo.text()
        birth = self.birthInfo.date().toPyDate()
        birth_str = birth.strftime('%Y-%m-%d')
        phone = self.phoneInfo.text()
        face_image = self.img_data
        office_num = self.officeInfo.text()
        company = self.companyInfo.text()
        card = self.cardInfo.text()
        if card == "":
            card = "0"
        
        self.user_data = {
                "user_name": name,
                "birth": birth_str,
                "phone_number": phone,
                "user_face": face_image,
                "office": office_num,
                "company": company,
                "rfid_UID": card
            }
        self.check_blank()
    
    def check_blank(self):
        is_blank = False
        exception_key = "rfid_UID"
        for key, value in self.user_data.items():
            if key != exception_key and value == "":
                is_blank = True
        if is_blank:
            self.blank_warning()
        else:
            
            self.regist_confirm_notice()
            
    def regist_confirm_notice(self):
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Question)
        confirmation.setText("이용자를 등록하시겠습니까?")
        confirmation.setWindowTitle("이용자 등록")
        confirmation.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        confirmation.buttonClicked.connect(self.regist_confirm)
        confirmation.exec_()

    def regist_confirm(self):
        try:
            self.db_connector.db_connect()
            self.db_connector.insert_value("UserInfo", self.user_data)
            self.img_data = ""
            
        except Exception as e:
            print(e)
        
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText("등록이 완료되었습니다.")
        confirmation.setWindowTitle("등록 완료")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.buttonClicked.connect(self.regist_complete)
        confirmation.exec_()
        
    def regist_complete(self, button):
        self.accept()
            
    def blank_warning(self):
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText("필수항목을 모두 입력해주세요")
        confirmation.setWindowTitle("blank check")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.exec_()    
            
    def add_user(self):
        try:
            self.db_connector.db_connect()
            self.db_connector.insert_value("UserInfo", self.user_data)
        except Exception as e:
            print(e)
        finally:
            self.regist_confirm_notice()
        
    def closeEvent(self, event):
        event.accept()
            

def main():
    rclpy.init()
    db_connector = DBConnector()
    db_manager = db_connector.db_manager
    app = QApplication(sys.argv)
    myWindow = AdminGUI()
    # myWindow = AdminGUI()    
    myWindow.show()
    
    signals = myWindow.signals
    executor = MultiThreadedExecutor()

    amcl_subscriber = AmclSubscriber(signals)
    path_subscriber = PathSubscriber(signals)
    request_subscriber = RequestSubscriber(signals)
    generate_qr_server = GenerateQRServer()
    order_subscriber = OrderSubscriber(myWindow)
    rfid_node = RFIDSubscriber(db_manager) 
    qr_check_server = QRCheckServer(signals)


    executor.add_node(amcl_subscriber)
    executor.add_node(path_subscriber)
    executor.add_node(request_subscriber)
    executor.add_node(generate_qr_server)
    executor.add_node(order_subscriber)
    executor.add_node(rfid_node)
    executor.add_node(qr_check_server)

    thread = threading.Thread(target=executor.spin)
    thread.start()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
