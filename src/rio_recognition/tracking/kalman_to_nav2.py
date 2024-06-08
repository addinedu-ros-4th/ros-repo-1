import rclpy
import numpy as np
import cv2
import threading
import time
from ultralytics import YOLO
from filterpy.kalman import KalmanFilter
from feature_extractor import FeatureExtractor
# from obj_tracking import TrackingController

class KalmanTracker(threading.Thread):
    def __init__(self, bbox, features):
        threading.Thread.__init__(self)
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        # 측정 noise 공분산 행렬
        self.kf.F = np.array([
            [1, 0, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 0, 1],
            [0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 1]
        ])
        # 오차 공분산 행렬
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0]
        ])

        self.kf.R *= 10 # 측정값의 불확실성
        self.kf.P *= 10 # 초기 상태의 불확실성
        self.kf.Q *= 0.01 # 시스템 모델의 불확실성

        # 추적할 객체의 초기 위치/속도 정의(초기 상태 벡터)
        self.kf.x[:4] = np.array(bbox).reshape((4, 1))
        self.features = features
        self.running = True

    def run(self):
        while self.running:
            self.kf.predict()
            time.sleep(0.01)

    def stop(self):
        self.running = False

    def update(self, bbox, features):
        self.kf.update(np.array(bbox).reshape((4, 1)))
        self.features = features

    def get_state(self):
        return self.kf.x[:4].reshape((4,))
    
    def get_features(self):
        return self.features

class ObjectDetector(threading.Thread):
    def __init__(self, model_path):
        threading.Thread.__init__(self)
        self.model = YOLO(model_path)
        self.names = self.model.model.names
        self.running = True

    def get_bounding_boxes(self, results):
        object_boxes = []
        classes = []

        for result in results:
            bounding_boxes = result.boxes
            for box in bounding_boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cls = int(box.cls[0])
                object_boxes.append([x1, y1, x2, y2])
                classes.append(cls)
        return object_boxes, classes

    def detect(self, frame, conf=0.4, vid_stride=30, max_det=100):
        results = self.model.predict(frame, conf=conf, vid_stride=vid_stride, max_det=max_det, verbose=False)
        if results[0].boxes is not None:
            return self.get_bounding_boxes(results)
        return [], []
    
    def run(self):
        while self.running:
            time.sleep(0.01)

    def stop(self):
        self.running = False

class ObjectTracker(threading.Thread):
    def __init__(self, model_path, object_name): #, tracking_controller
        threading.Thread.__init__(self)
        self.detector = ObjectDetector(model_path)
        self.feature_extractor = FeatureExtractor()
        self.object_name = object_name
        self.tracker = None
        self.initial_detection = True
        self.tracking_object_id = None
        self.tracked_object_features = None
        self.last_known_position = None
        self.colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
        self.running = True
        # self.tracking_controller = tracking_controller
        self.track_distance = None
        self.area = None
        self.found_match = True

    def run(self):
        cap = cv2.VideoCapture(0)
        self.detector.start()

        while self.running:
            ret, frame = cap.read()
            if not ret:
                break

            frame_height, frame_width = frame.shape[:2]
            x_start = frame_width // 6
            x_end = 5 * frame_width // 6
            roi_frame = frame[:, x_start:x_end]

            object_boxes, cls_indices = self.detector.detect(frame)

            if self.initial_detection:
                largest_box = None
                largest_area = 0

                for idx, (object_box, cls_index) in enumerate(zip(object_boxes, cls_indices)):
                    object_id = idx
                    class_name = self.detector.names[cls_index]

                    x1, y1, x2, y2 = object_box
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height

                    if area > largest_area:
                        largest_area = area
                        largest_box = [x1, y1, width, height]
                        self.tracking_object_id = object_id

                if largest_box is not None:
                    roi = frame[largest_box[1]:largest_box[1]+largest_box[3], largest_box[0]:largest_box[0]+largest_box[2]]
                    features = self.feature_extractor.extract_features(roi)
                    self.tracker = KalmanTracker(largest_box, features)
                    self.tracker.start()
                    self.initial_detection = False
                    self.tracked_object_features = features
                    
            else:
                if self.tracker:
                    tracked_box = self.tracker.get_state()
                    x, y, x2, y2 = tracked_box
                    cv2.rectangle(frame, (int(x), int(y)), (int(x2), int(y2)), (0, 0, 255), 2)
                    cv2.putText(frame, self.object_name, (int(x), int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                    found_matching_object = False
                    min_distance = float('inf')
                    best_box = None
                    best_features = None

                    for object_box in object_boxes:
                        x1, y1, x2, y2 = object_box
                        center_x = x1 + (x2 - x1) / 2
                        center_y = y1 + (y2 - y1) / 2
                        tracked_center_x = x + (x2 - x) / 2
                        tracked_center_y = y + (y2 - y) / 2

                        distance = np.sqrt((center_x - tracked_center_x) ** 2 + (center_y - tracked_center_y) ** 2)

                        if distance < 50: # 추적 중인 객체의 이전 좌표와 현재 인식한 객체와의 거리(거리가 상대적으로 먼 객체는 추적중인 객체 후보에서 제외)
                            roi = frame[y1:y2, x1:x2]
                            features = self.feature_extractor.extract_features(roi)
                            feature_distance = np.linalg.norm(features - self.tracker.get_features())
                            print("특징 벡터 차이 : ", feature_distance)

                            if feature_distance < 500: # 추적 중인 객체의 특징 벡터와 현재 인식한 객체의 특징 벡터와의 차이
                                if feature_distance + distance < min_distance:
                                    min_distance = feature_distance + distance
                                    best_box = [x1, y1, x2, y2]
                                    best_features = features
                                    found_matching_object = True

                    if found_matching_object:
                        self.found_match = True
                        self.tracker.update(best_box, best_features)
                        self.tracked_object_features = best_features
                    else:
                        self.found_match = False
                        self.last_known_position = tracked_box
                        self.tracked_object_features = self.tracker.get_features()
                        self.tracker.stop()
                        self.tracker.join()
                        self.tracker = None
                        # self.tracking_controller.is_control_active = False
                        # self.is_control_active = False

                        print("Object lost... Last known position : ", self.last_known_position)
                        cv2.putText(frame, "unknown", (int(tracked_box[0]), int(tracked_box[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                
                else:
                    min_feature_distance = float('inf')
                    best_box = None
                    best_features = None
                    for object_box in object_boxes:
                        x1, y1, x2, y2 = object_box
                        roi = frame[y1:y2, x1:x2]
                        features = self.feature_extractor.extract_features(roi)
                        feature_distance = np.linalg.norm(features - self.tracked_object_features)
                        if feature_distance < min_feature_distance:
                            min_feature_distance = feature_distance
                            best_box = [x1, y1, x2, y2]
                            best_features = features

                    if best_box is not None and min_feature_distance < 500:
                        self.tracker = KalmanTracker(best_box, best_features)
                        self.tracker.start()

            if self.tracker:
                bbox = self.tracker.get_state().tolist()

                width = bbox[2] - bbox[0]
                height = bbox[3] - bbox[1]
                self.area = width * height

                center_x = (bbox[0] + bbox[2]) / 2  
                center_y = (bbox[1] + bbox[3]) / 2
                frame_center_x = frame_width / 2
                frame_center_y = frame_height / 2
                track_distance = np.sqrt((center_x - frame_center_x) ** 2 + (center_y - frame_center_y) ** 2)

                if center_x > frame_center_x:
                    self.track_distance = track_distance
                elif center_x < frame_center_x:
                    self.track_distance = -track_distance

                # self.tracking_controller.set_box_info(self.distance, self.area)
                # self.set_box_info(distance, obj_area)

            cv2.imshow('Frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        self.detector.stop()
        self.detector.join()
        if self.tracker:
            self.tracker.stop()
            self.tracker.join()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":

    rclpy.init()

    # tracking_controller = TrackingController()
    # executor = rclpy.executors.MultiThreadedExecutor()  
    # executor.add_node(tracking_controller)

    model_path = 'ultralytics/human_detection/best.pt'
    object_name = 'wook'
    object_tracker = ObjectTracker(model_path, object_name) # , tracking_controller
    object_tracker.start()

    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass

    object_tracker.running = False
    object_tracker.join()

    rclpy.shutdown()
