import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from FaceRecognition import FaceRecognition
from cv_bridge import CvBridge
bridge = CvBridge()

class ImageSubscriber(Node):
    def __init__(self):
        self.facerecognition = FaceRecognition()
        super().__init__('image_sub')
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.face_names_pub = self.create_publisher(String, '/face_names', 10)

    def image_callback(self, data) :
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        labeled_image, face_names = self.facerecognition.name_labeling(cv_image, show_result=False)
        cv2.imshow('face_recognition', labeled_image)
        cv2.waitKey(33)

        # 얼굴 이름을 문자열로 변환하여 발행
        face_names_str = ','.join(face_names)
        self.face_names_pub.publish(String(data=face_names_str))

def main(args=None) :
    rp.init(args=args)
    node = ImageSubscriber()

    node.facerecognition.add_known_face("/home/joe/ros-repo-1/src/rio_recognition/face_recog/data/ho_0.jpg", "joe")

    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.get_logger().info('KeyboardInterrupt')
    finally :
        node.destroy_node()
        rp.shutdown()

if __name__ == '__main__' :
    main()
