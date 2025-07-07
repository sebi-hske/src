#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from ImageController import ImageController

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.video_path = "/home/sebi/ros2_ws/RobotikProjektSoSe2025/Videos/video_marker_erkennbar.mp4"
        
        
        self.get_logger().info("CameraNode mit Live-Feed gestartet.")
        self.loop()

    def loop(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            print("⚠️ Could not load video.")
            exit()
        cv2.namedWindow("marker", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("marker", 960, 540)
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("⚠️ Failed to grab frame")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
            parameters = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(aruco_dict, parameters)

            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None:
                self.get_logger().error("No Markers detected")
            else:
                #print(ids)
                pass
                
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
                self.get_logger().warn("Specified Markers not found")
            else:
                src_pts = np.array([detected_points[i] for i in ordered_ids], dtype="float32")

                cropped, x_min, y_min = ImageController.crop_bounding_rect(gray, src_pts)
                warped = ImageController.rectify_cropped_image(cropped, src_pts, x_min, y_min)
                mirrored = cv2.flip(warped, 0)
                print(src_pts)
                cropped_image = ImageController.crop(mirrored, 0, 0, 86, 36)
          
                
            
                cv2.imshow("marker", cropped_image)
            key = cv2.waitKey(30)
            if key == 27 or key == ord('q'):
                break
            
        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        
        rclpy.shutdown()

if __name__ == '__main__':  
    main()