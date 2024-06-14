import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
import cv2
from FaceRecognition import FaceRecognition
import threading
import socket
import struct
import numpy as np
import time
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import queue

bridge = CvBridge()

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_sub')
        self.facerecognition = FaceRecognition()
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw_1',
            self.image_callback,
            10
        )

        self.face_names_pub = self.create_publisher(String, '/face_names_1', 10)
        self.face_landmarks_pub = self.create_publisher(String, '/face_landmarks', 10)

        self.srv = self.create_service(SetBool, 'register_service', self.handle_register_service)

    def image_callback(self, data):
        self.cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        labeled_image, face_names = self.facerecognition.name_labeling(self.cv_image, show_result=False)
        cv2.imshow('face_recognition', labeled_image)
        cv2.waitKey(33)

        face_names_str = ','.join(face_names)
        self.face_names_pub.publish(String(data=face_names_str))

    def handle_register_service(self, request, response):
        if request.data:
            latest_image = self.cv_image
            self.publish_landmark(latest_image)
            response.success = True
            response.message = "Landmarks published"
        else:
            response.success = False
            response.message = "No action taken"
        return response

    def publish_landmark(self, image):
        face_landmarks = self.facerecognition.get_landmark(image)
        face_landmarks_str = str(face_landmarks)
        self.face_landmarks_pub.publish(String(data=face_landmarks_str))
        

class FaceRecognitionThread(threading.Thread):
    def __init__(self, face_recognition, face_names_pub):
        super().__init__()
        self.face_recognition = face_recognition
        self.face_names_pub = face_names_pub
        self.frame_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.stop_thread = threading.Event()

    def run(self):
        while not self.stop_thread.is_set():
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
                labeled_image, face_names = self.face_recognition.name_labeling(frame, show_result=False)
                face_names_str = ','.join(face_names)
                self.face_names_pub.publish(String(data=face_names_str))
                print(face_names_str)
                self.result_queue.put(labeled_image)

    def add_frame(self, frame):
        if not self.frame_queue.full():
            self.frame_queue.put(frame)

    def get_result(self):
        if not self.result_queue.empty():
            return self.result_queue.get()
        return None

    def stop(self):
        self.stop_thread.set()

class IPSubscriber(Node):
    def __init__(self):
        super().__init__('ip_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ip_address',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ip_address = None

    def listener_callback(self, msg):
        self.get_logger().info(f'Received IP Address: {msg.data}')
        self.ip_address = msg.data

class TCPIPClientNode(Node):
    def __init__(self, ip_address):
        super().__init__('tcpip_client_node')
        self.client_socket = None
        self.connection = None
        self.connect_to_server(ip_address)
        self.receive_thread = threading.Thread(target=self.receive_video)
        self.receive_thread.start()
        self.facerecognition = FaceRecognition()
        self.ip_address = ip_address

        self.face_names_pub = self.create_publisher(String, '/face_names_1', 10)
        
        self.face_recognition_thread = FaceRecognitionThread(self.facerecognition, self.face_names_pub)
        self.face_recognition_thread.start()

    def connect_to_server(self, ip_address):
        while True:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((ip_address, 5607))
                self.connection = self.client_socket.makefile('rb')
                self.get_logger().info(f"Connected to server at {ip_address}")
                break
            except Exception as e:
                self.get_logger().error(f"Failed to connect to server at {ip_address}: {e}")
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
                        # cv2.imshow('face_recognition', frame)
                        # cv2.waitKey(33)
                        
                        # 얼굴 인식 스레드에 프레임 추가
                        self.face_recognition_thread.add_frame(frame)
                        
                        # 얼굴 인식 결과 가져오기
                        # result = self.face_recognition_thread.get_result()
                        # if result is not None:
                        #     cv2.imshow('labeled_face_recognition', result)
                        #     cv2.waitKey(33)
                    else:
                        self.get_logger().error("Invalid frame received")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                self.reconnect_to_server() # 재연결 시도
            # finally:
            #     if self.connection:
            #         self.connection.close()
            #     if self.client_socket:
            #         self.client_socket.close()
            #     cv2.destroyAllWindows()
            
    def reconnect_to_server(self):
        # 서버와 연결이 끊긴 경우 재연결 시도
        self.get_logger().warning("Lost connection to server. Reconnecting...")
        self.client_socket.close()
        self.connection.close()
        self.connect_to_server(self.ip_address)

    def destroy(self):
        self.face_recognition_thread.stop()
        self.face_recognition_thread.join()

def main(args=None):
    rp.init(args=args)

    ip_address = "192.168.123.106"

    tcpip_client_node = TCPIPClientNode(ip_address)
    
    tcpip_client_node.facerecognition.add_known_face("./data/wooks/wook_0.jpg", "wook")
    tcpip_client_node.facerecognition.add_known_face("./data/kyus/kyu_0.jpg", "kyu")
    tcpip_client_node.facerecognition.add_known_face("./data/joes/ho_0.jpg", "joe")

    try:
        rp.spin(tcpip_client_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        tcpip_client_node.destroy()

        rp.shutdown()

if __name__ == '__main__':
    main()