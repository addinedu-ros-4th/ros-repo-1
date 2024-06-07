import cv2
import numpy as np
import os
from skimage.feature import graycomatrix, graycoprops
from skimage.io import imread
from skimage.color import rgb2gray
from scipy.spatial import distance
        
class FeatureExtractor:
    def __init__(self, bins=(8, 8, 8)):
        self.bins = bins

    def extract_color_histogram(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1, 2], None, self.bins, [0, 180, 0, 256, 0, 256])
        hist = cv2.normalize(hist, hist).flatten()
        return hist

    def extract_haralick_features(self, image):
        gray = rgb2gray(image)
        glcm = graycomatrix((gray * 255).astype('uint8'), distances=[1], angles=[0, np.pi/4, np.pi/2, 3*np.pi/4], symmetric=True, normed=True)
        haralick_features = []
        properties = ['contrast', 'dissimilarity', 'homogeneity', 'energy', 'correlation', 'ASM']
        for prop in properties:
            haralick_features.append(graycoprops(glcm, prop).flatten())
        haralick_features = np.concatenate(haralick_features)
        return haralick_features

    def extract_features(self, image):
        color_histogram = self.extract_color_histogram(image)
        haralick_features = self.extract_haralick_features(image)
        return np.concatenate([color_histogram, haralick_features])

class ObjectRecognizer:
    def __init__(self, image_folder, extractor):
        self.extractor = extractor
        self.object_features = self._extract_features(image_folder)

    def _extract_features(self, image_folder):
        features = {}
        for image_name in os.listdir(image_folder):
            image_path = os.path.join(image_folder, image_name)
            image = imread(image_path)
            features[image_name] = self.extractor.extract_features(image)
        return features

    def calculate_similarity(self, features1, features2):
        return distance.euclidean(features1, features2)

    def recognize(self, frame):
        webcam_features = self.extractor.extract_features(frame)
        similarities = {image_name: self.calculate_similarity(webcam_features, features) for image_name, features in self.object_features.items()}
        return min(similarities, key=similarities.get)

if __name__ == "__main__":
    image_folder = '/home/wook/amr_ws/project/final/user_recognition/siamese_test/data_opt/Human'
    feature_extractor = FeatureExtractor()
    recognizer = ObjectRecognizer(image_folder, feature_extractor)

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        most_similar_object = recognizer.recognize(frame)
        cv2.putText(frame, f'Most similar object: {most_similar_object}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Webcam', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()