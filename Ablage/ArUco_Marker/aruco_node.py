import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2 as cv
import numpy as np
import os

class ArucoDistance(Node):

    def __init__(self):
        super().__init__('aruco_dist_pub')
        self.publisher_distance_to_marker = self.create_publisher(Float32, 'aruco_distance', 10)
        self.publisher_marker_id = self.create_publisher(Float32, 'aruco_id', 10)
        self.cap = cv.VideoCapture(0)
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.camera_matrix = None  # Placeholder for camera matrix
        self.distortion_coefficients = None  # Placeholder for distortion coefficients

        timer_period = 0.2  # Publishes data every 0.2 seconds (5Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load calibration data and set camera matrix and distortion coefficients
        self.load_calibration_data()

    def load_calibration_data(self):
        calibration_data_path = '/home/sebi/ros2_ws/src/Ablage/ArUco_Marker/calibration.npz'
        if not os.path.exists(calibration_data_path):
            self.get_logger().warning(f"Error: Calibration file '{calibration_data_path}' not found.")
            return

        self.calibration_data = np.load(calibration_data_path)
        self.camera_matrix = self.calibration_data['camera_matrix']
        self.distortion_coefficients = self.calibration_data['distortion_coefficients']

    def timer_callback(self):
        ret, frame = self.cap.read()  # Capture frame from the camera
        if not ret:
            self.get_logger().warning('No frame received.')
            return

        distance_marker = self.calculate_distance_to_marker(frame) #Publishs the Distance to the  Marker, if an aruco marker is detected
        if distance_marker is not None:
            self.publish_distance_marker(distance_marker)

        marker_id = self.read_marker_id()  
        if marker_id is not None:
            self.publish_marker_id(marker_id)

    def calculate_distance_to_marker(self, frame):
        img_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, self.ids, _ = cv.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

        if corners is not None and len(corners) > 0:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners, 7, self.camera_matrix, self.distortion_coefficients
            )
            distance = np.sqrt(tvecs[0][0][2] ** 2 + tvecs[0][0][0] ** 2 + tvecs[0][0][1] ** 2)
            return distance/100
        else:
            return -1.0
        

    def read_marker_id(self):

        if self.ids is not None:
            marker_id = self.ids
            return marker_id
        else:
            return -1.0       
       
        
    def publish_distance_marker(self, distance_marker):
        msg_marker = Float32()
        msg_marker.data = float(distance_marker)
        self.publisher_distance_to_marker.publish(msg_marker)
        self.get_logger().info('Publishing distance to marker: "%s"' % msg_marker.data)
        
    def publish_marker_id(self, marker_id):
        msg_id = Float32()
        msg_id.data = float(marker_id)
        self.publisher_marker_id.publish(msg_id)
        self.get_logger().info('Publishing Marker IDs: "%s"' % msg_id.data)

def main(args=None):
    rclpy.init(args=args)
    aruco_dist_pub = ArucoDistance()
    rclpy.spin(aruco_dist_pub)
    aruco_dist_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()