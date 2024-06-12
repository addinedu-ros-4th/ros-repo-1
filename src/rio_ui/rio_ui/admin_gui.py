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
import numpy as np

from example_interfaces.msg import Int64MultiArray
from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from rio_ui.admin_service import *


admin_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "admin_service.ui")
add_user_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "add_user.ui")
office_manage_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "office_manage.ui")
admin_ui = uic.loadUiType(admin_file)[0]
add_user_ui = uic.loadUiType(add_user_file)[0]
office_manage_ui = uic.loadUiType(office_manage_file)[0]

robot_colors = {
        'deliverybot': Qt.blue,
        'guidebot': Qt.darkGreen,
        'cleanerbot': Qt.gray,
        'patrolbot': Qt.red,
        'minibot': Qt.magenta
    }

space_loc_info = {
        'minibot_charger_1' : [-25.42, -0.82],
        'test_1' : [-25.29, -1.28]
    }

class AdminGUI(QMainWindow, admin_ui):
    def __init__(self):    
        super().__init__()
        self.setupUi(self)
        # self.btn_req_guide.clicked.connect(lambda: self.robot_ctl_task("guidebot"))
        # self.btn_req_delivery.clicked.connect(lambda: self.robot_ctl_task("deliverybot"))
        # self.btn_req_patrol.clicked.connect(lambda: self.robot_ctl_task("patrolbot"))
        # self.btn_req_clean.clicked.connect(lambda: self.robot_ctl_task("cleanerbot"))
        self.init_robot_info()

       
        self.addUserBtn.clicked.connect(self.add_user)
        self.officeManageBtn.clicked.connect(self.office_manage)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_robot_status)
        self.timer.start(10)
        
        self.order = []
        self.node = rclpy.create_node("robot_task_node_deli")
        self.task_publisher_deli = self.node.create_publisher(Int64MultiArray, "/robot_task_deli", 10)
        
        self.node = rclpy.create_node("robot_task_node_guide")
        self.task_publisher_guide = self.node.create_publisher(Int64MultiArray, "/robot_task_guide", 10)

        # self.tts = TTSAlertService()
        # self.tts.run_tts("admin_greeting")

        self.db_connector = DBConnector()
        # self.db_manager = db_manager
        tables = self.db_connector.show_all_tables()
        self.select_table_update(tables)
        self.selectRobotTask.currentIndexChanged.connect(self.request_table_update)
        self.detail_bt.clicked.connect(self.table_detail)
        
        self.signals = ROSNodeSignals()
        self.robot_states = {}
        self.signals.amcl_pose_received.connect(self.update_amcl_pose)
        # self.signals.path_distance_received.connect(self.update_path_distance)
        # self.signals.task_request_received.connect(self.update_task_request)
        self.signals.visitor_alert_received.connect(self.visitor_alert_to_user)
        # self.signals.service_signal_received.connect(self.publish_robot_service)

        with open(os.path.join(get_package_share_directory("rio_main"), "maps", "office.yaml")) as f:
            map_data = yaml.full_load(f)

        with Image.open(os.path.join(get_package_share_directory("rio_main"), "maps", "office.pgm")) as f:
            map_pgm = np.array(f)

        self.h, self.w = map_pgm.shape
        qimage = QImage(map_pgm.tobytes(), self.w, self.h, self.w, QImage.Format_Grayscale8)

        self.map_resolution = map_data["resolution"]
        self.map_origin = map_data["origin"][:2]

        self.pixmap = QPixmap.fromImage(qimage)
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()

        self.pixmap = self.pixmap.scaled(self.Map_label.width(), self.Map_label.height())
        self.Map_label.setPixmap(self.pixmap)
        self.Map_label.setAlignment(Qt.AlignCenter)
        
        header = self.requestTable.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)

        self.task_requester = TaskRequester()
        # self.init_robot_info()

    # def init_robot_info(self):
    #     self.robot_locations = {
    #         'guidebot': {
    #             'prev': [0.0, 0.0],
    #             'pres' : [0.0, 0.0]
    #         },
    #         'deliverybot': {
    #             'prev': [0.0, 0.0],
    #             'pres' : [0.0, 0.0]
    #         },
    #         'patrolbot': {
    #             'prev': [0.0, 0.0],
    #             'pres' : [0.0, 0.0]
    #         },
    #         'cleanerbot': {
    #             'prev': [0.0, 0.0],
    #             'pres' : [0.0, 0.0]
    #         },
    #         'minibot': {
    #             'prev': [0.0, 0.0],
    #             'pres' : [0.0, 0.0]
    #         }
    #     }

    #     self.robot_states_uiEdit = {
    #         'guidebot': [self.robot_cn_guide, self.robot_ts_guide, self.lineEdit],
    #         'deliverybot': [self.robot_cn_delivery, self.robot_ts_delivery, self.lineEdit_2],
    #         'patrolbot': [self.robot_cn_patrol, self.robot_ts_patrol, self.lineEdit_3],
    #         'cleanerbot': [self.robot_cn_clean, self.robot_ts_clean, self.lineEdit_4],
    #         'minibot': [self.yLine, self.yawLine, self.xLine]
    #     }

    def init_robot_info(self):
        self.btn_submit_task.clicked.connect(self.click_robot_ctl_task)
        self.btn_stop_robot.clicked.connect(self.click_robot_task_stop)

        self.robot_task_stack = {
            'guidebot': [],
            'deliverybot': [],
            'patrolbot': [],
            'cleanerbot': [],
            'minibot': []
        }

        self.robot_last_task = {
            'guidebot': "",
            'deliverybot': "",
            'patrolbot': "",
            'cleanerbot': "",
            'minibot': ""
        }
        
        self.robot_task_info = {
            'guidebot': [],
            'deliverybot': [],
            'patrolbot': [],
            'cleanerbot': [],
            'minibot': []
        }
        
        self.service_info = {
            'guidebot': [],
            'deliverybot': [],
            'patrolbot': [],
            'cleanerbot': [],
            'minibot': []
        }

        last_task = self.load_last_task()
        for robot, task in last_task.items():
            self.robot_last_task[robot] = task

        # self.robot_states_uiEdit = {
        #     'guidebot': [self.robot_cn_guide, self.robot_ts_guide, self.location_guide, False],
        #     'deliverybot': [self.robot_cn_delivery, self.robot_ts_delivery, self.location_delivery, False],
        #     'patrolbot': [self.robot_cn_patrol, self.robot_ts_patrol, self.location_patrol, False],
        #     'cleanerbot': [self.robot_cn_clean, self.robot_ts_clean, self.location_clean, False],
        #     'minibot': [self.yLine, self.yawLine, self.xLine, False]
        # }

    def load_last_task(self):
        with open(os.path.join(get_package_share_directory("rio_ui"), "data", "last_task.yaml"), 'r') as f:
            last_task = yaml.full_load(f)
        return last_task
    
    def save_last_task(self, data):
        with open(os.path.join(get_package_share_directory("rio_ui"), "data", "last_task.yaml"), 'w') as f:
            yaml.dump(data, f)
            
    def request_table_update(self):
        select_robot_type = self.selectRobotTask.currentText()
        task_list = self.robot_task_info[select_robot_type]
        try:
            for row_position in range(len(task_list)):
                for i, value in enumerate():
                    self.ui.requestTable.setItem(row_position, i, QTableWidgetItem(value))
        except Exception as e:
            print(e)


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

    def update_amcl_pose(self, robot_states):
        self.robot_states = robot_states
        

    # def update_path_distance(self, distance):
    #     if distance < 0.4:
    #         distance = 0
    #     self.remainLine.setText(str("{:.2f}".format(distance)))

    # def update_task_request(self, x, y, yaw):
    #     self.xLine.setText(str(x))
    #     self.yLine.setText(str(y))
    #     self.yawLine.setText(str(yaw))
    #     self.robot_ctl_task()

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
        

    def click_robot_task_stop(self):
        selected = self.robot_type.currentText()
        if selected not in robot_types:
            print("[WARN] Select Robot First.")
            return
        else:
            robot = selected

        flag = self.robot_states_uiEdit[robot][-1]
        btn = self.btn_stop_robot
        btn.setStyleSheet("font-weight: bold;")
        if not flag:
            flag = True
            btn.setText("RESUME\nROBOT")
            mode = "pause"
        else:
            flag = False
            btn.setText("STOP\nROBOT")
            mode = "moving"
        self.robot_states_uiEdit[robot][-1] = flag

        last_task = self.load_last_task()
        for r, t in last_task.items():
            self.robot_last_task[r] = t

        if self.robot_states[robot]['task_id'] != '':
            task_id = self.robot_states[robot]['task_id']
        else:
            task_id = self.robot_last_task[robot]

        self.robot_last_task[robot] = task_id

        # print(task_id)
        self.task_stop(robot, mode, task_id)

    def task_stop(self, robot, mode, task_id):
        self.task_requester.mode_change(robot, mode, task_id)

        data = self.load_last_task()
        data[robot] = task_id
        # print(data)
        self.save_last_task(data)

    def click_robot_ctl_task(self):
        selected = self.robot_type.currentText()

        if selected not in robot_types:
            print("[WARN] Select Robot First.")
            return
        else:
            robot = selected

            if robot == "guidebot":
                robot = "minibot" # minibot test

            goal = self.task_goal_loc.currentText()
            self.dispatch_task(robot, goal)

    def dispatch_task(self, robot, goal):
        params = {
            'fleet': robot,  
            'robot': robot + "_1", 
            'place': goal,
        }
        self.task_requester.task_msg_pub(params)

        task = [params, False]
        self.robot_task_stack[robot].append(task)

        # print(self.robot_task_stack[robot])

    def update_robot_status(self):
        self.Map_label.setPixmap(self.pixmap)
        self.Map_label.setAlignment(Qt.AlignCenter)
        painter = QPainter(self.Map_label.pixmap())
        for robot, states in self.robot_states.items():
            self.update_robot_health(robot, states, painter)
        painter.end()

    def update_robot_health(self, robot, states, painter):
        if states['connection'] >= 20:
            self.update_map(robot, states, painter)
            self.robot_states_uiEdit[robot][0].setStyleSheet("color: green;")
            self.robot_states_uiEdit[robot][1].setStyleSheet("color: green;")
            self.robot_states_uiEdit[robot][0].setText("Connected")

            # TODO location name print
            self.robot_states_uiEdit[robot][2].setText(str(f"{states['x']:.2f} / {states['y']:.2f}"))
            
            status = self.update_task_progress(robot, states)
            self.robot_states_uiEdit[robot][1].setText(status)

        elif 0 < states['connection'] < 20:
            self.robot_states_uiEdit[robot][0].setStyleSheet("color: blue;")
            self.robot_states_uiEdit[robot][0].setText("Connecting...")
        else:
            self.robot_states_uiEdit[robot][0].setStyleSheet("color: red;")
            self.robot_states_uiEdit[robot][0].setText("Disconnected...")
            self.robot_states_uiEdit[robot][2].setText("")

    def update_task_progress(self, robot):
        status = f"{robot['progress']}"
        return status

    def update_map(self, robot, states, painter):
        label_w_r = self.pixmap.width() 
        label_h_r = self.pixmap.height()

        x, y = self.calc_coord(states['x'], states['y'])
        posx = x * label_w_r  /100
        posy = label_h_r - abs(y * label_h_r / 100)

        if robot in robot_colors:
            pen_color = robot_colors[robot]
        else:
            pen_color = Qt.black

        painter.setPen(QPen(pen_color, 15, Qt.SolidLine))
        painter.drawPoint(int(posx), int(posy))
        painter.drawText(int(posx-2), int(posy-11), str(robot))

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
        add_user_window = AddUserGUI()
        add_user_window.exec_()
        
    def office_manage(self):
        office_manage_window = OfficeManagerGUI()
        office_manage_window.exec_()
                
    def closeEvent(self, event):
        self.timer.stop()
        if self.db_connector and self.db_connector.db_manager:
            self.db_connector.db_manager.close()

        for robot in robot_types:
            self.robot_last_task[robot] = self.robot_states[robot]['task_id']

        self.save_last_task(self.robot_last_task)

        rclpy.shutdown()
        # self.tts.stop_tts()
        event.accept()
        
        
class OfficeManagerGUI(QDialog, office_manage_ui):
    def __init__(self):
        super().__init__()
        
        self.setupUi(self)
        self.db_connector = DBConnector()
        self.password.setEchoMode(QLineEdit.Password) 
        self.passwordCheck.setEchoMode(QLineEdit.Password)
        self.openDate.setCalendarPopup(True)
        self.openDate.setDisplayFormat("yyyy-MM-dd")
        self.openDate.setDate(QDate.currentDate())
        self.closeDate.setCalendarPopup(True)
        self.closeDate.setDisplayFormat("yyyy-MM-dd")
        self.closeDate.setDate(QDate.currentDate())
        
        table = "OfficeInfo"
        column = "office_number"
        criteria = "close_date"
        
        result = self.db_connector.select_specific_null(table, column, criteria)
        for value in result:
            self.officeNumCBX.addItem(str(value['office_number'])) 
        self.change_company_label()

        
        
        self.officeNumCBX.currentIndexChanged.connect(self.change_company_label)
        self.addBtn.clicked.connect(self.read_info)
        self.applyBtn.clicked.connect(self.close_office_confirm)
        self.cancel_bt.clicked.connect(self.close)
        self.cancel_bt_3.clicked.connect(self.close)
        
    def change_company_label(self):
        table = "OfficeInfo"
        column = "company_name"
        current_office = {"office_number": self.officeNumCBX.currentText()}
        company = self.db_connector.select_specific(table, column, current_office)[0]
        self.current_company = str(company["company_name"])
        self.companyLabel.setText(self.current_company)
        
    def read_info(self):
        office_num = self.officeNum.text()
        company = self.companyName.text()
        password = self.password.text()
        password_check = self.passwordCheck.text()
        open_date = self.openDate.date().toPyDate()
        open_date_str = open_date.strftime('%Y-%m-%d')
        
        if password == password_check:
            self.office_data = {
                    "office_number": office_num,
                    "company_name": company,
                    "password": password,
                    "open_date": open_date_str
                }
            self.check_blank()
            
        else:
            self.password_match_warning()
            
    def password_match_warning(self):
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText("비밀번호가 일치하지 않습니다")
        confirmation.setWindowTitle("password check")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.exec_()  
        
    def check_blank(self):
        is_blank = False
        exception_key = "close_date"
        for key, value in self.office_data.items():
            if key != exception_key and value == "":
                is_blank = True
                
        if is_blank:
            self.blank_warning()
        else:
            
            self.regist_confirm_notice()  
            
    def regist_confirm_notice(self):
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Question)
        confirmation.setText("사무실 이용현황을 등록하시겠습니까?")
        confirmation.setWindowTitle("사무실 등록")
        confirmation.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        confirmation.buttonClicked.connect(self.regist_confirm)
        confirmation.exec_()
        
    def regist_confirm(self):
        try:
            self.db_connector.db_connect()
            self.db_connector.insert_value("OfficeInfo", self.office_data)
            
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
        
    def close_office_confirm(self):
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Question)
        confirmation.setText("해당 회사를 퇴점처리 하시겠습니까?")
        confirmation.setWindowTitle("사무실 퇴점")
        confirmation.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        confirmation.buttonClicked.connect(self.close_office)
        confirmation.exec_()
        
    def close_office(self):
        office_num = self.officeNumCBX.currentText()
        close_date = self.openDate.date().toPyDate()
        close_date_str = close_date.strftime('%Y-%m-%d')
        
        table = "OfficeInfo"
        data = {"close_date": close_date_str}
        criteria ={"office_number": office_num, "company_name": self.current_company}
        self.db_connector.update_value(table, data, criteria)  
        
        confirmation = QMessageBox()
        confirmation.setIcon(QMessageBox.Information)
        confirmation.setText("퇴점이 완료되었습니다.")
        confirmation.setWindowTitle("퇴점 완료")
        confirmation.setStandardButtons(QMessageBox.Ok)
        confirmation.buttonClicked.connect(self.close_complete)
        confirmation.exec_()     
        
    def close_complete(self, button):
        self.accept() 
        
        
        
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
        pass
        
    def closeEvent(self, event):
        event.accept()
            

def main():
    rclpy.init()
    db_connector = DBConnector()
    db_manager = db_connector.db_manager
    app = QApplication(sys.argv)
    myWindow = AdminGUI()
    myWindow.show()
    
    signals = myWindow.signals
    executor = MultiThreadedExecutor()

    amcl_subscriber = AmclSubscriber(signals)
    # path_subscriber = PathSubscriber(signals)
    request_subscriber = RobotCallSubscriber(myWindow)
    generate_qr_server = GenerateQRServer()
    order_subscriber = OrderSubscriber(myWindow)
    rfid_node = RFIDSubscriber(db_manager) 
    qr_check_server = QRCheckServer(signals)


    executor.add_node(amcl_subscriber)
    # executor.add_node(path_subscriber)
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
