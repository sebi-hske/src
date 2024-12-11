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
        self.publisher_marker_center = self.create_publisher(Float32, 'center_offset', 10)
        self.cap = cv.VideoCapture(0)
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
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

        marker_id = self.ids
        if marker_id is not None:
            self.publish_marker_id(marker_id)

    def calculate_distance_to_marker(self, frame):
        img_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, self.ids, _ = cv.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)
        self.calculate_center_offset(corners, self.ids)
        if corners is not None and len(corners) > 0:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners, 7, self.camera_matrix, self.distortion_coefficients
            )
            distance = np.sqrt(tvecs[0][0][2] ** 2 + tvecs[0][0][0] ** 2 + tvecs[0][0][1] ** 2)
            return distance/100
        else:
            return -1.0
        
    def calculate_center_offset(self, corners, ids):
        marker_centers = []
        if len(corners) > 0:
          # Flatten the ArUco IDs list
          #ids = ids.flatten()

          # Loop over the detected ArUco corners
          for (marker_corner, marker_id) in zip(corners, ids):

            # Extract the marker corners
            corners = marker_corner.reshape((4, 2))
            (top_left, top_right, bottom_right, bottom_left) = corners

            # Convert the (x,y) coordinate pairs to integers
            top_right = (int(top_right[0]), int(top_right[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
            top_left = (int(top_left[0]), int(top_left[1]))

            # Calculate the center of the ArUco marker
            center_x = int((top_left[0] + bottom_right[0]) / 2.0)
            center_y = int((top_left[1] + bottom_right[1]) / 2.0)
            marker_centers.append((center_x, center_y))            

        # If more than one marker is detected, calculate the midpoint between the first two    
        if len(marker_centers) >= 2:
          # Calculate midpoint
          midpoint_x =  int((marker_centers[0][0] + marker_centers[1][0]) / 2)
          midpoint_y = int((marker_centers[0][1] + marker_centers[1][1]) / 2)    
          self.get_logger().info('Center between Markers at: '+ str(midpoint_x) + ' | ' + str(midpoint_y))        

    def publish_center_offset(self, center_offset):
        msg_offset = Float32()
        msg_offset.data = float(center_offset)
        self.publisher_marker_center.publish(center_offset)
        self.get_logger().info('Publishing offset to center: "%s"' % msg_offset.data)

    def publish_distance_marker(self, distance_marker):
        msg_marker = Float32()
        msg_marker.data = float(distance_marker)
        self.publisher_distance_to_marker.publish(msg_marker)
        self.get_logger().info('Publishing distance to marker: "%s"' % msg_marker.data)
        
    def publish_marker_id(self, marker_id):
        marker_id = marker_id.flatten()
        marker_id = np.asarray(marker_id, int)
        msg_list = marker_id.tolist()
        
        for val in msg_list:
            if val == 999.0:
                print('Node terminated via ID: 999')
                exit(0)
            else:    
                msg_id = Float32()
                msg_id.data = float(val)
                self.publisher_marker_id.publish(msg_id)
                self.get_logger().info('Publishing Marker IDs: "%s"' % msg_id.data)
        

def main(args=None):
    rclpy.init(args=args)
    try:
        aruco_dist_pub = ArucoDistance()
        rclpy.spin(aruco_dist_pub)
        aruco_dist_pub.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()