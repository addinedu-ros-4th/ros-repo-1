import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
import cv2
from FaceRecognition import FaceRecognition
from cv_bridge import CvBridge

bridge = CvBridge()

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_sub')
        self.facerecognition = FaceRecognition()
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.face_names_pub = self.create_publisher(String, '/face_names', 10)
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

def main(args=None):
    rp.init(args=args)
    face_recog = ImageSubscriber()

    
    face_recog.facerecognition.add_known_face("./data/wooks/wook_0.jpg", "wook")
    face_recog.facerecognition.add_known_face("./data/kyus/kyu_0.jpg", "kyu")
    face_recog.facerecognition.add_known_face("./data/joes/ho_0.jpg", "joe")

    try:
        rp.spin(face_recog)
    except KeyboardInterrupt:
        face_recog.get_logger().info('KeyboardInterrupt')
    finally:
        face_recog.destroy_face_recog_node()
        rp.shutdown()

if __name__ == '__main__':
    main()