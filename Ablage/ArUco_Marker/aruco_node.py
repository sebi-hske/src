import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String
import cv2 as cv
import numpy as np
import os

MARKERSIZE = 70                 #immer beide werte beachten!!
DISTANCE_COEFFICIANT = 10000    #_V_V_V_V_V_V_V_V_V_V_V_V_V_
CALIBRATION_DATA_PATH = '/home/sebi/ros2_ws/src/Ablage/ArUco_Marker/calibration.npz'
ARUCO_DICT = cv.aruco.DICT_4X4_1000

class ArucoDistance(Node):

    def __init__(self):
        super().__init__('aruco_dist_pub')
        self.publisher_id_and_dst = self.create_publisher(String, 'id_dst', 10)             #bei beiden topics evtl pfade erstellen
        self.publisher_marker_center = self.create_publisher(Float32, 'center_offset', 10)  

        self.cap = cv.VideoCapture(0)                                                       
        self.aruco_dict = cv.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = cv.aruco.DetectorParameters()
        self.camera_matrix = None  # Placeholder for camera matrix
        self.distortion_coefficients = None  # Placeholder for distortion coefficients

        timer_period = 0.2  # Publishes data every 0.2 seconds (5Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)       #start loop  

        # Load calibration data and set camera matrix and distortion coefficients
        self.load_calibration_data()

    def load_calibration_data(self):
        
        if not os.path.exists(CALIBRATION_DATA_PATH):
            self.get_logger().warning(f"Error: Calibration file '{CALIBRATION_DATA_PATH}' not found.")
            return

        self.calibration_data = np.load(CALIBRATION_DATA_PATH)
        self.camera_matrix = self.calibration_data['camera_matrix']
        self.distortion_coefficients = self.calibration_data['distortion_coefficients']

    def timer_callback(self):
        ret, frame = self.cap.read()  # Capture frame from the camera
        if not ret:
            self.get_logger().warning('No frame received.')
            return
        
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)   #optimize workload by converting img to greyscale
        distance_marker = self.calculate_distance_to_marker(frame) #Publishs the Distance to the  Marker, if an aruco marker is detected
        #self.soft_stop(self.marker_tuple[1])
        dst_tuple = (distance_marker, )                       
        
        self.marker_tuple += dst_tuple                  #workaround combination both tuples

        marker_id = self.marker_tuple[1]                
        if marker_id is not None:
            self.publish_id_and_dst(self.marker_tuple)

        
        
    def calculate_distance_to_marker(self, frame):
        corners, ids, _ = cv.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        self.marker_tuple = (corners, ids)        
        if self.marker_tuple[0] is not None and len(self.marker_tuple[0]) > 0:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                self.marker_tuple[0], MARKERSIZE, self.camera_matrix, self.distortion_coefficients
            )
            distance = np.sqrt(tvecs[0][0][2] ** 2 + tvecs[0][0][0] ** 2 + tvecs[0][0][1] ** 2)
            offset_single_marker = tvecs[0][0][0]
            self.calculate_center_offset(frame, offset_single_marker)
            return distance / DISTANCE_COEFFICIANT      
        else:
            return None
        
    def calculate_center_offset(self, frame, offset_single_marker):
        marker_centers = []
        image_width = frame.shape[1]
        center_x = offset_single_marker + image_width
        #print(frame.shape)
        if len(self.marker_tuple[1]) > 1:
            # Flatten the ArUco IDs list
            #ids = ids.flatten()
            #print('2 marker')
            # Loop over the detected ArUco corners
            for (marker_corner, marker_id) in zip(self.marker_tuple[0], self.marker_tuple[1]):  
              # Extract the marker corners
              marker_corner = marker_corner.reshape((4, 2))
              (top_left, top_right, bottom_right, bottom_left) = marker_corner  
              # Convert the (x,y) coordinate pairs to integers
              top_right = (int(top_right[0]), int(top_right[1]))
              bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
              bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
              top_left = (int(top_left[0]), int(top_left[1]))   
              # Calculate the center of the ArUco marker
              center_x = int((top_left[0] + bottom_right[0]) / 2.0)
              center_y = int((top_left[1] + bottom_right[1]) / 2.0)
              marker_centers.append((center_x, center_y))
              #print(len(marker_centers))            

        # If more than one marker is detected, calculate the midpoint between the first two    
        if len(marker_centers) >= 2:
            # Calculate midpoint
            midpoint_x =  int((marker_centers[0][0] + marker_centers[1][0]) / 2)
            center_x = midpoint_x - image_width // 2
            midpoint_y = int((marker_centers[0][1] + marker_centers[1][1]) / 2)
            #self.get_logger().info('Center between Markers at: '+ str(midpoint_x) + ' | ' + str(midpoint_y))image_width = frame.shape[1]       
        
        if center_x is not None:
            self.publish_center_offset(center_x)   
      

    def publish_center_offset(self, center_offset):        
        msg_offset = Float32()
        msg_offset.data = float(center_offset)
        self.publisher_marker_center.publish(msg_offset)
        #self.get_logger().info('Publishing offset to center: "%s"' % msg_offset.data)

    def publish_id_and_dst(self, data_tuple):
        (_, id, dst) = data_tuple   #corners wird nicht verwendet            
        msg_id_dst = String()
        msg_id_dst.data = str(id.item(0)) + ", " + str(dst)
        self.publisher_id_and_dst.publish(msg_id_dst)
        #self.get_logger().info('publishing combined id and distance: "%s"' % msg_id_dst.data)
        

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
    