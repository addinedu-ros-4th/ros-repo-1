from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.msg import Int64MultiArray
from threading import Thread
import datetime

from rio_ui.admin_service import *

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
        self.user_id = msg.data[0]
        self.change_total = msg.data[1]
        self.user_credit = msg.data[2]
        self.ID_check()
        
        
    def ID_check(self):
        uid_list = []
        detail_data = self.db_manager.read("Payment")
        try:
            for data in detail_data:  
                uid = data.get('rfid_uid')  
                if uid is not None:  
                    uid_list.append(uid)

            if self.user_id in uid_list:
                current_credit = self.calc_total(self.user_id)
                self.user_credit = current_credit - self.change_total
                self.data[0] = 1
                self.data[1] = self.user_id
                self.data[2] = self.user_credit
                
                
            else:
                self.data[0] = 0
                self.data[1] = self.user_id
                self.data[2] = 0
            
            msg = Int64MultiArray(data = self.data)
            self.publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error check UID: {e}')
            
    def calc_total(self, id):
        try:
            total_list = []
            criteria = {"rfid_uid": id}
            change_total_log = self.db_manager.read("Payment", criteria, "payment_id")
            for data in change_total_log:
                total = data.get("total_credit")
                total_list.append(total)
            current_credit = total_list[-1]    
            return current_credit
        except Exception as e:
            self.get_logger().error(f'Error check UID: {e}')
            

def main():
    rclpy.init()
    db_connector = DBConnector()
    db_manager = db_connector.db_manager
    rfid_node = RFIDSubscriber(db_manager) 
    rclpy.spin(rfid_node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()