import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2 as cv
import numpy as np
import os

MARKERSIZE = 0.07                 #immer beide werte beachten!!
DISTANCE_COEFFICIANT = 100    #_V_V_V_V_V_V_V_V_V_V_V_V_V_
CALIBRATION_DATA_PATH = '/home/sebi/ros2_ws/src/ar_pipe_server/ar_pipe_server/calibration.npz'
ARUCO_DICT = cv.aruco.DICT_4X4_1000

class ArucoDistance(Node):

    def __init__(self):
        super().__init__('aruco_dist_pub')
        self.publisher_all_data = self.create_publisher(String, 'data', 10)             #bei beiden topics evtl pfade erstellen
     
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
        
        dst_tuple = (distance_marker, )                       
        
        self.marker_tuple += dst_tuple                  #workaround combination both tuples
                                                        #tuple with corners, ids, offset, distance
        marker_id = self.marker_tuple[1]                
        if marker_id is not None:
            self.publish_id_offset_dst(self.marker_tuple)

        
        
    def calculate_distance_to_marker(self, frame):
        corners, ids, _ = cv.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        self.marker_tuple = (corners, ids)        
        if self.marker_tuple[0] is not None and len(self.marker_tuple[0]) > 0:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(                                       #opencv version auf roboter??
                self.marker_tuple[0], MARKERSIZE, self.camera_matrix, self.distortion_coefficients
            )
            distance = np.sqrt(tvecs[0][0][2] ** 2 + tvecs[0][0][0] ** 2 + tvecs[0][0][1] ** 2)
            offset_single_marker = (tvecs[0][0][0],)
            self.marker_tuple += offset_single_marker
           
            return distance #/ DISTANCE_COEFFICIANT      
        else:
            return None
      

    def publish_id_offset_dst(self, data_tuple):
        (_, id, offset, dst) = data_tuple   #corners wird nicht verwendet            
        msg_id_dst = String()
        msg_id_dst.data = str(id.item(0)) + " " + str(offset) + " " + str(dst)
        self.publisher_all_data.publish(msg_id_dst)
        #self.get_logger().info('id, offset, distance: "%s"' % msg_id_dst.data)
        

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
    