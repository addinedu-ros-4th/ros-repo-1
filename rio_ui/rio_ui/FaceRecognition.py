import matplotlib.pyplot as plt
import numpy as np
import cv2
import face_recognition

class FaceRecognition:
    def __init__(self):
        self.known_face_encodings = []
        self.known_face_names = []
        self.known_face_locations = []

    def plt_imshow(self, title='image', img=None, figsize=(8, 5)):
        if isinstance(img, np.ndarray):
            if len(img.shape) < 3:
                rgbImg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            else:
                rgbImg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            plt.imshow(rgbImg)
            plt.title(title)
            plt.xticks([]), plt.yticks([])
            plt.show()

    def get_landmark(self, input_image):
        face_landmarks = face_recognition.face_landmarks(input_image)
        return face_landmarks

    def name_labeling(self, input_image, show_result=True):
        image = input_image.copy()
        face_locations = face_recognition.face_locations(image)
        face_encodings = face_recognition.face_encodings(image, face_locations)

        face_names = []

        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding, tolerance=0.5)
            name = "Unknown"

            if matches:
                face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                if len(face_distances) > 0:
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = self.known_face_names[best_match_index]

            face_names.append(name)

        for (top, right, bottom, left), name in zip(face_locations, face_names):
            color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
            cv2.rectangle(image, (left, top), (right, bottom), color, 1)
            cv2.rectangle(image, (left, bottom - 10), (right, bottom), color, cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(image, name, (left + 3, bottom - 3), font, 0.2, (0, 0, 0), 1)

        # if show_result:
        #     self.plt_imshow("Output", image, figsize=(24, 15))

        return image, face_names

    def draw_label(self, input_image, coordinates, label):
        image = input_image.copy()
        (top, right, bottom, left) = coordinates
        cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 5)
        cv2.putText(image, label, (left - 10, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
        return image

    def add_known_face(self, face_image, name):
        if face_image is None:
            print(f"Error: Unable to load image from {name}")
            return
        face_locations = face_recognition.face_locations(face_image)
        if not face_locations:
            print(f"Error: No face detected in image {name}")
            return
        face_encodings = face_recognition.face_encodings(face_image, face_locations)
        if not face_encodings:
            print(f"Error: Unable to encode face in image {name}")
            return

        detected_face_image = self.draw_label(face_image, face_locations[0], name)
        self.known_face_encodings.append(face_encodings[0])
        self.known_face_names.append(name)
        self.known_face_locations.append(face_locations[0])
        
        print(f"Added known face: {name}")

if __name__ == "__main__":
    face_recog = FaceRecognition()

    face_recog.add_known_face("wooks/wook_0.jpg", "wook")
    face_recog.add_known_face("kyus/kyu_0.jpg", "kyu")

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("웹캠에서 영상을 불러올 수 없습니다.")
            break

        labeled_frame = face_recog.name_labeling(frame, show_result=False)

        cv2.imshow('Webcam', labeled_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
