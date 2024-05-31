from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *
import sys
import os
import yaml

from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from rio_ui.admin_service import *

ui_file = os.path.join(get_package_share_directory("rio_ui"), "ui", "admin_service.ui")
admin_ui = uic.loadUiType(ui_file)[0]


class AdminGUI(QMainWindow, admin_ui):
    def __init__(self):    
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
        
        self.db_connector = DBConnector()
        # self.db_manager = db_manager
        tables = self.db_connector.show_all_tables()
        self.select_table_update(tables)
        self.detail_bt.clicked.connect(self.table_detail)
        
        self.signals = ROSNodeSignals()
        self.signals.amcl_pose_received.connect(self.update_amcl_pose)
        self.signals.path_distance_received.connect(self.update_path_distance)
        self.signals.task_request_received.connect(self.update_task_request)

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
    
    def closeEvent(self, event):
        self.timer.stop()
        if self.db_connector and self.db_connector.db_manager:
            self.db_connector.db_manager.close()
        rclpy.shutdown()
        event.accept()

def main():
    rclpy.init()
    # db_manager = DBConnector()
    app = QApplication(sys.argv)
    # myWindow = AdminGUI(db_manager)
    myWindow = AdminGUI()    
    myWindow.show()
    
    signals = myWindow.signals
    executor = MultiThreadedExecutor()

    amcl_subscriber = AmclSubscriber(signals)
    path_subscriber = PathSubscriber(signals)
    request_subscriber = RequestSubscriber(signals)
    user_service_server = UserService()


    executor.add_node(amcl_subscriber)
    executor.add_node(path_subscriber)
    executor.add_node(request_subscriber)
    executor.add_node(user_service_server)

    thread = threading.Thread(target=executor.spin)
    thread.start()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
