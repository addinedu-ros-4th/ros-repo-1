import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.msg import Int64MultiArray
from mfrc522 import SimpleMFRC522
from threading import Thread
import RPi.GPIO as GPIO
from rfid_reader.servo import ServoController
import time
import threading

class RFIDReaderNode(Node):
    def __init__(self):
        super().__init__('rfid_reader')
        
        GPIO.setwarnings(False)
        

        self.reader = SimpleMFRC522()
        self.servo_controller = ServoController()
        self.get_logger().info('RFID Reader Node has been started.')
        self.publisher = self.create_publisher(Int64MultiArray, "/rfid_info", 10)
        self.id = None
        self.is_first_tag = True
        self.is_charging = False
        self.schedule_read_rfid()

        # ROS2 타이머를 사용하여 주기적으로 RFID 읽기
    def schedule_read_rfid(self):
        threading.Timer(1, self.read_rfid).start()

    def read_rfid(self):
        try:
            if not self.is_charging:
                id = self.reader.read_id()
                if id and id != self.id:
                    self.id, point = self.reader.read()
                    self.get_logger().info(f"RFID ID: {self.id}, Text: {point}")
                    cleaned_string = point.replace('\0', '')
                    
                    if cleaned_string == "":
                        point = 0
                    else: 
                        point = int(point)
                    msg = Int64MultiArray()
                    msg.data = [self.id, point]
                    self.publisher.publish(msg)
                    self.id = id
            
        except Exception as e:
            self.get_logger().error(f'Error reading RFID: {e}')
            self.is_charging = False

    def destroy_node(self):
        self.servo_controller.cleanup()
        GPIO.cleanup()
        super().destroy_node()

class RFIDSubscriber(Node):
    def __init__(self, rfid_node):
        super().__init__("rfid_sub")
        self.rfid_node = rfid_node
        self.subscription = self.create_subscription(
            Int64MultiArray,
            "/rfid_renew",
            self.rfid_callback,
            10)
        self.check_id = False
        GPIO.setmode(GPIO.BCM)
        self.servo_controller = ServoController()
        self.servo_controller.set_angle(0)

    def rfid_callback(self, msg):
        try:
            self.rfid_node.is_charging = True
            if msg.data[0]:
                self.check_id = True
                
            uid = msg.data[1]
            point = str(msg.data[2])
            
            id = self.rfid_node.reader.read_id()
            if id == uid and self.check_id:
                self.servo_controller.set_angle(90)
                
                
                print("처리가 완료되었습니다.")
                print(point)
                
            print("정상 처리 되었습니다!")
            self.rfid_node.is_charging = False
            
        except Exception as e:
            self.get_logger().error(f'Error reading RFID: {e}')
            self.rfid_node.is_charging = False

def main(args=None):
    rclpy.init()

    executor = MultiThreadedExecutor()

    rfid_reader = RFIDReaderNode()
    rfid_subs = RFIDSubscriber(rfid_reader)

    executor.add_node(rfid_reader)
    executor.add_node(rfid_subs)

    thread = Thread(target=executor.spin)
    thread.start()

    try:
        rclpy.spin_until_future_complete(rfid_reader.get_logger().get_clock().create_future())
    finally:
        thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()