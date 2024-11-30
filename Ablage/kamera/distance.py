import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2 as cv
import numpy as np
import os

class ArucoDistancePublisher(Node):

    def __init__(self):
        super().__init__('aruco_distance_publisher')
        self.publisher_distance_to_robot = self.create_publisher(Float32, 'aruco_distance', 10)
        self.publisher_distance_to_line = self.create_publisher(Float32, 'line_distance', 10)
        self.cap = cv.VideoCapture(0)
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_250)
        self.aruco_params = cv.aruco.DetectorParameters_create()
        self.camera_matrix = None  # Placeholder for camera matrix
        self.distortion_coefficients = None  # Placeholder for distortion coefficients

        timer_period = 0.2  # Publishes data every 0.2 seconds (5Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load calibration data and set camera matrix and distortion coefficients
        self.load_calibration_data()

    def load_calibration_data(self):
        calibration_data_path = '/home/ubuntu/kamera/calib_data/calibration.npz'
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

        distance_robot = self.calculate_distance_to_robot(frame) #Publishs the Distance to the Robot ahead, if an aruco marker is detected
        if distance_robot is not None:
            self.publish_distance_robot(distance_robot)

        distance_line = self.calculate_distance_to_line(frame)  #Publishs the Distance to the Line, if a line is detected
        if distance_line is not None:
            self.publish_distance_line(distance_line)

    def calculate_distance_to_robot(self, frame):
        img_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, _, _ = cv.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

        if corners is not None and len(corners) > 0:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners, 7, self.camera_matrix, self.distortion_coefficients
            )
            distance = np.sqrt(tvecs[0][0][2] ** 2 + tvecs[0][0][0] ** 2 + tvecs[0][0][1] ** 2)
            return distance/100
        else:
            return -1.0
    def calculate_distance_to_line(self, image):

        # Funktion zum Umwandeln des Bildes in ein Binärbild
        def convert_to_binary(image):
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            _, binary_image = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
            return binary_image

        # Funktion zur Durchführung der Segmentierung
        def segmentation(image):
            contours, _ = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            segmented_image = np.zeros_like(image)
            cv.drawContours(segmented_image, contours, -1, (255), thickness=cv.FILLED)
            return segmented_image, contours

        # Funktion zur Durchführung der morphologischen Transformation
        def morphological_transform(image):
            kernel = np.ones((5, 5), np.uint8)
            transformed_image = cv.morphologyEx(image, cv.MORPH_CLOSE, kernel)
            return transformed_image

        # Funktion zum Abschneiden der unteren zwei Drittel des Bildes
        def crop_lower_two_thirds(image, cut_percentage=50):
            height, width = image.shape[:2]
            lower_percentage = cut_percentage
            lower_two_thirds = int(lower_percentage * height / 100)
            return image[lower_two_thirds:, :]

        cropped_image = crop_lower_two_thirds(image)

        smoothed_image = cv.GaussianBlur(cropped_image, (7, 7), 0)

        # 2. Umwandeln des Bildes in ein Binärbild
        binary_image = convert_to_binary(smoothed_image)

        # 4. Durchführung der morphologischen Transformation
        transformed_image = morphological_transform(binary_image)

        smoothed_image = cv.GaussianBlur(transformed_image, (7, 7), 0)

        # 3. Durchführung der Segmentierung und Erhalt der Konturen
        segmented_image, contours = segmentation(smoothed_image)

        # Sortiere Konturen nach ihrer Fläche in absteigender Reihenfolge
        contours = sorted(contours, key=cv.contourArea, reverse=True)

        if len(contours) > 0:
            # Wähle die größte Kontur
            largest_contour = contours[0]
            
            # Berechne den Schwerpunkt (Centroid) der Kontur
            M = cv.moments(largest_contour)
            if(M["m00"] != 0):
                centroid_x = int(M["m10"] / M["m00"])

                # Berechne die horizontale Abweichung vom Bildmittelpunkt
                image_width = image.shape[1]
                return centroid_x - image_width // 2
            else:
                return 66666
            
        else:
            return 66666
        
    def publish_distance_robot(self, distance_robot):
        msg_robot = Float32()
        msg_robot.data = float(distance_robot)
        self.publisher_distance_to_robot.publish(msg_robot)
        self.get_logger().info('Publishing distance to robot: "%s"' % msg_robot.data)
        
    def publish_distance_line(self, distance_line):
        msg_line = Float32()
        msg_line.data = float(distance_line)
        self.publisher_distance_to_line.publish(msg_line)
        self.get_logger().info('Publishing distance to line: "%s"' % msg_line.data)

def main(args=None):
    rclpy.init(args=args)
    aruco_distance_publisher = ArucoDistancePublisher()
    rclpy.spin(aruco_distance_publisher)
    aruco_distance_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()