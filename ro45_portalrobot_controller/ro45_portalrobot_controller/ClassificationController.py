#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import joblib
from functools import lru_cache

class ClassifcationController:

    @staticmethod
    def extract_features_from_image(image):
        if image is None or image.size == 0:
            return None
        contours, _ = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area == 0:
            return None
        perimeter = cv2.arcLength(cnt, True)
        compactness = (perimeter**2) / (4 * np.pi * area)
        M = cv2.moments(cnt)
        hu = cv2.HuMoments(M).flatten()
        if np.any(np.isnan(hu)) or np.isnan(compactness):
            return None
        hu = -np.sign(hu) * np.log10(np.abs(hu) + 1e-10)
        hu1 = hu[0]
        return [float(compactness), float(hu1)]

    @staticmethod
    def _load_model(model_path="/home/sebi/ros2_ws/src/ro45_portalrobot_controller/ro45_portalrobot_controller/svm_model_compactness_hu1.pkl"):
        return joblib.load(model_path)

    @staticmethod
    def classify_image(image, id: int, model_path="/home/sebi/ros2_ws/src/ro45_portalrobot_controller/ro45_portalrobot_controller/svm_model_compactness_hu1.pkl"):
        features = ClassifcationController.extract_features_from_image(image)
        if features is None:
            return 0

        model = ClassifcationController._load_model(model_path)
        X_new = np.array(features).reshape(1, -1)
        prediction = model.predict(X_new)[0]
        if prediction == "Einhorn":
            return 1, id
        elif prediction == "Katze":
            return 2, id
        elif prediction == "Rest":
            return 0, id