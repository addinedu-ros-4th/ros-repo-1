from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.msg import Int64MultiArray
from threading import Thread

from rio_db_manager.db_manager import DBManager
from rio_db_manager.create_init_db import CreateInitDB


class RFIDSubscriber(Node):
    def __init__(self, db_manager = None):
        super().__init__("rfid_sub")
        self.data = [0, 0, 0]
        self.subscription = self.create_subscription(
            Int64MultiArray,
            "/rfid_info",
            self.info_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Int64MultiArray,
            "/rfid_renew",
            10
        )
        
        self.db_manager = db_manager
        
        
    def info_callback(self, msg):
        self.user_id = msg.data[0]
        self.user_point = msg.data[1]
        self.ID_check()
        

    def ID_check(self):
        uid_list = []
        detail_data = self.db_manager.read("Payment")
        try:
            for data in detail_data:  # 각 딕셔너리에 대해 반복
                uid = data.get('rfid_uid')  # 'rfid_UID' 키에 해당하는 값을 가져옴
                if uid is not None:  # 값이 None이 아니면 uid_list에 추가
                    # uid_int = int.from_bytes(uid, byteorder='little', signed=False)
                    uid_list.append(uid)
            print(uid_list)        
            if self.user_id in uid_list:
                self.data[0] = 1
                self.data[1] = self.user_id
                self.data[2] = self.user_point
                print("---------")
                
            else:
                self.data[0] = 0
                self.data[1] = self.user_id
                self.data[2] = self.user_point
            msg = Int64MultiArray(data = self.data)
            self.publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error check UID: {e}')
 
        
        
def main():
    rclpy.init()
    db_manager = DBManager("db_config.yaml")
    rfid_node = RFIDSubscriber(db_manager) 
    rclpy.spin(rfid_node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()