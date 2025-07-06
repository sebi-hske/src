#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import ImageController

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher_ = self.create_publisher(Image, 'processed_image', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Kamera konnte nicht geöffnet werden.")
            return

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.get_logger().info("CameraNode mit Live-Feed gestartet.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Kein Frame von Kamera erhalten.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None:
            return

        marker_corner_indices = {
            0: 3, 1: 2, 2: 1, 3: 0
        }

        detected_points = {}
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in marker_corner_indices:
                corner_index = marker_corner_indices[marker_id]
                selected_corner = corners[i][0][corner_index]
                detected_points[marker_id] = selected_corner.copy()

        ordered_ids = [3, 2, 1, 0]
        if not all(mid in detected_points for mid in ordered_ids):
            return

        src_pts = np.array([detected_points[i] for i in ordered_ids], dtype="float32")

        try:
            cropped, x_min, y_min = ImageController.ImageController.crop_bounding_rect(gray, src_pts)
            warped = ImageController.ImageController.rectify_cropped_image(cropped, src_pts, x_min, y_min)
            mirrored = cv2.flip(warped, 0)

            cropped_image = ImageController.ImageController.crop(mirrored, 0, 0, 86, 36)

            kernel = np.ones((3, 3), np.uint8)
            cleaned = cv2.morphologyEx(cropped_image, cv2.MORPH_OPEN, kernel)

            threshold_image = cv2.threshold(cleaned, 160, 255, cv2.THRESH_BINARY)

            display = cv2.cvtColor(threshold_image, cv2.COLOR_GRAY2BGR)
            h, w = display.shape[:2]

            ros_image = self.bridge.cv2_to_imgmsg(display, encoding="bgr8")
            self.publisher_.publish(ros_image)

        except Exception as e:
            self.get_logger().warn(f"Verarbeitung fehlgeschlagen: {e}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':  
    main()