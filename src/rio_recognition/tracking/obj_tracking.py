import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kalman_to_nav2 import *

class TrackingController(Node):
    def __init__(self, object_tracker):
        super().__init__('obj_tracking_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.height = 240
        self.width = 320
        self.is_control_active = True
        self.alpha = 0.1 # lpf 계수(0~1)
        self.alpha_angle = 0.1 
        self.filtered_obj_area = 0.0
        self.filtered_angular_speed = 0.0
        self.object_tracker = object_tracker
        # self.distance = distance
        # self.obj_area = area

    def timer_callback(self):
        if self.is_control_active:
            distance, obj_area = self.get_box_info()

            if distance is None or obj_area is None:
                return
            
            self.filtered_obj_area = self.alpha * obj_area + (1 - self.alpha) * self.filtered_obj_area

            if -30 < distance < 30 and obj_area == 80000:
                self.is_control_active = False
                self.stop_control()
            else:
                twist = Twist()
                self.control_speed(twist, obj_area)
                self.control_direction(twist, distance)
                self.publisher_.publish(twist)
                

    # def get_box_info(self):
    #     return self.distance, self.obj_area

    # def set_box_info(self, distance, obj_area):
    #     self.distance = distance
    #     self.obj_area = obj_area

    def get_box_info(self):
        distance = self.object_tracker.track_distance
        obj_area = self.object_tracker.area
        return distance, obj_area
    
    def control_direction(self, twist, distance):

        max_angular_speed = 2.0
        min_angular_speed = 0.1

        angular_speed = min_angular_speed + (max_angular_speed - min_angular_speed) * abs(distance) / 320.0
        self.filtered_angular_speed = self.alpha_angle * angular_speed + (1 - self.alpha_angle) * self.filtered_angular_speed

        if self.object_tracker.found_match:
            if distance < -30:
                twist.angular.z = self.filtered_angular_speed
            elif distance > 30:
                twist.angular.z = -self.filtered_angular_speed
        else:
            last_box = self.object_tracker.last_known_position
            if (last_box[2] + last_box[0])/2 > 160:
                twist.angular.z = -self.filtered_angular_speed
            else:
                twist.angular.z = self.filtered_angular_speed

        self.get_logger().info(f'Controlling direction: distance={distance}, angular.z={twist.angular.z}')

    def control_speed(self, twist, distance, obj_area):
        
        max_speed = -5.0
        min_speed = -0.5

        max_area = 7000
        min_area = 1000

        if obj_area > max_area:
            speed = min_speed
        elif obj_area < min_area:
            speed = max_speed
        else:
            speed = min_speed + (max_speed - min_speed) * (max_area - obj_area) / (max_area - min_area)

        twist.linear.x = speed   
        self.get_logger().info(f'Controlling robot: distance={distance}, obj_area={obj_area}')

    def stop_control(self):
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Target conditions met')
        self.is_control_active = True
    
def main(args=None):
    rclpy.init(args=args)

    model_path = '../ultralytics/human_detection/best.pt'
    object_name = 'wook'

    object_tracker = ObjectTracker(model_path, object_name) # , tracking_controller
    object_tracker.start()

    tracking_controller = TrackingController(object_tracker)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tracking_controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    object_tracker.running = False
    object_tracker.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()