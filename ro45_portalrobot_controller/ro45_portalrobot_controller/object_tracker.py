import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ro45_action_interfaces.action import Intercept
from ro45_portalrobot_controller.motion_predictor import MotionPredictor
from ro45_portalrobot_controller.ClassificationController import ClassifcationController
from ro45_portalrobot_controller.ImageController import ImageController



    
class TrackerClient(Node):
    def __init__(self):
        super().__init__('tracking_client')
        self.action_client = ActionClient(self, Intercept, 'intercept_object')
        self.next_id = 1
        self.tracked_objects = []
        self.classified_objects = set()
        self.predictor = MotionPredictor()
        self.classificator = ClassifcationController()
        #self.video_path = "/home/sebi/ros2_ws/RobotikProjektSoSe2025/Videos/video_marker_erkennbar.mp4"
        self.video_path = 2
        self.size_img_to_markers()
        self.loop()  # Start the tracker loop

    def size_img_to_markers(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            print("⚠️ Could not load video.")
            exit()
        cv2.namedWindow("Cropping Check", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Cropping Check", 960, 540)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("⚠️ Failed to grab frame")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None:
                self.get_logger().error("No Markers detected")
                cv2.imshow("Cropping Check", gray)
            else:
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
                    self.src_pts = np.array([detected_points[i] for i in ordered_ids], dtype="float32")
                    

                    cropped_image = self.call_image_controller(gray)
                    
                    cv2.putText(cropped_image, 
                                    f'Check if framing ok, press Q to continue, press E to exit', 
                                    (20, 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                
                    cv2.imshow("Cropping Check", cropped_image)
                key = cv2.waitKey(30)
                if key == ord('e'):
                    exit()
                if key == 27 or key == ord('q'):
                    break
            
        cap.release()
        cv2.destroyAllWindows()

    def send_goal(self, x, y, time, obj_class):
        self.get_logger().info(f"Sending goal")
        goal_msg = Intercept.Goal()
        goal_msg.position_x = float(x)
        goal_msg.position_y = float(y)
        goal_msg.time = time
        goal_msg.object_class = obj_class
    
        if not self.action_client.wait_for_server(5.0):
            self.get_logger().error("Action server not available. Cannot send goal.")
            return False
        try:       
            goal_future = self.action_client.send_goal_async(goal_msg)
            return True
            
        except Exception as e:
            self.get_logger().error(f'Move failed with error: {str(e)}')
            return False
        

    def find_id(self, cx, cy, current_time):
        distance_threshold_px = 80
        object_timeout_s = 60.0
        min_dist = float('inf')
        best_index = None
        for i, obj in enumerate(self.tracked_objects):
            dist = math.hypot(cx - obj['x'], cy - obj['y'])
            dt = current_time - obj['time']
            if dist < distance_threshold_px and dt < object_timeout_s:
                if dist < min_dist:
                    min_dist = dist
                    best_index = i
        if best_index is not None:
            obj = self.tracked_objects[best_index]
            obj.update({'x': cx, 'y': cy, 'time': current_time})
            return obj['id']
        new_id = self.next_id
        print(f"New ID {new_id} created at x={cx}, y={cy}")
        self.next_id += 1
        self.tracked_objects.append({'id': new_id, 'x': cx, 'y': cy, 'time': current_time})
        return new_id

    def remove_stale_objects(self,now):
        object_timeout_s = 60.0
        self.tracked_objects[:] = [obj for obj in self.tracked_objects if now - obj['time'] <= object_timeout_s]
        current_ids = {obj['id'] for obj in self.tracked_objects}
        for obj_id in list(self.predictor.position_history.keys()):
            if obj_id not in current_ids:
                self.predictor.position_history.pop(obj_id, None)
                self.predictor.prediction_given.discard(obj_id)

    def crop_image(self, image, crop_left, crop_right, crop_top, crop_bottom):
        height, width = image.shape[:2]
        start_x = crop_left
        end_x = width - crop_right
        start_y = crop_top
        end_y = height - crop_bottom
        return image[start_y:end_y, start_x:end_x]
    

    def count_corners(self, contour):
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        return len(approx)
    
    def extract_object_image(self, frame, contour, obj_id):
        x, y, w, h = cv2.boundingRect(contour)
        
        padding = int(min(w, h) * 0.1)
        x = max(0, x - padding)
        y = max(0, y - padding)
        w = min(frame.shape[1] - x, w + 2*padding)
        h = min(frame.shape[0] - y, h + 2*padding)
        
        roi = frame[y:y+h, x:x+w].copy()        
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        return roi
    
    def call_image_controller(self, gray):
        cropped, x_min, y_min = ImageController.crop_bounding_rect(gray, self.src_pts)
        warped = ImageController.rectify_cropped_image(cropped, self.src_pts, x_min, y_min)
        mirrored = cv2.flip(warped, 0)
            
        return ImageController.crop(mirrored, 10, 0, 45, 35)
    

    def loop(self):
        mm_per_pixel = 0.714
        self.px_to_gantry_factor = 0.0007
        
        min_area = 2800
        max_area = 3800
        min_corners = 6
        seconds_until_grip = 0.0
        grip_x_px = 700 
        grip_x_pos = 0.19
        grip_y_pos = 0.06
        middle_px = 90
        max_y = 0.085
        min_y = 0.04
        
        """28 px pro 20mm auf bild (kästchen schachbrett)
        von anfang WKS zum idealen greifpunkt ca 48cm
        das entspricht einem pixelwert von 728 angenommen mm und px skalieren linear 
        da entzerren nötig ist vermutlich nicht
        410px entsprechen dem Nullpunkt der X-Achse im WKS
        weitere anpassungen im Betrieb nötig"""
        grip_x_mm = grip_x_px * mm_per_pixel

        speed_zone_left = 60
        speed_zone_right = 170
        speed_zone_top = 0
        speed_zone_bottom = 160
        
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            print("⚠️ Could not load video.")
            exit()

        
        cv2.namedWindow("Object Detection + Prediction", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Object Detection + Prediction", 1510, 820)

        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("⚠️ Failed to grab frame")
                continue

            now = time.time()
            self.remove_stale_objects(now)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            frame_cropped = self.call_image_controller(gray)       
            
            _, thresh = cv2.threshold(frame_cropped, 120, 255, cv2.THRESH_BINARY)

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if not (min_area < area < max_area):
                    continue
                if self.count_corners(contour) < min_corners:
                    continue

                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"]) 

                #if not (speed_zone_left < cx < speed_zone_right and speed_zone_top < cy < speed_zone_bottom):
                #    continue

                # Bereichsprüfung für Geschwindigkeitsmessung
                in_speed_zone = (speed_zone_left <= cx <= speed_zone_right and
                                speed_zone_top <= cy <= speed_zone_bottom)

                obj_id = self.find_id(cx, cy, now)
                x_mm = cx * mm_per_pixel
                self.predictor.update(obj_id, x_mm, now, in_speed_zone)

                # Vorhersage nur nach Verlassen des Messbereichs
                if not in_speed_zone and obj_id not in self.predictor.prediction_given:
                    vx = self.predictor.compute_velocity_from_first_last(obj_id)
                    arrival_time = self.predictor.predict_arrival_time(obj_id, now, grip_x_mm)

                    if vx is not None and arrival_time is not None:
                        seconds_until_grip = arrival_time - now
                        x_pred_px = grip_x_px 
                        y_pred_px = cy

                        cv2.circle(frame_cropped, (x_pred_px, y_pred_px), 8, (255, 0, 0), 2)
                        cv2.putText(frame_cropped, f"Grab in {seconds_until_grip:.1f}s", (x_pred_px + 10, y_pred_px),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        print(f"Prediction: ID {obj_id} | Velocity = {vx:.2f} mm/s | ETA = {seconds_until_grip:.2f}s")
                        self.predictor.prediction_given.add(obj_id)
                        y_factor = (cy - middle_px) * self.px_to_gantry_factor
                        grip_y_pos += y_factor
                        print(grip_y_pos)

                        if grip_y_pos >= max_y:
                            grip_y_pos = max_y
                        elif grip_y_pos <= min_y:
                            grip_y_pos = min_y
                        self.send_goal(grip_x_pos, grip_y_pos, seconds_until_grip, erg[0])
                    

                if obj_id not in self.classified_objects:        
                    extracted_img = self.extract_object_image(thresh, contour, obj_id)
                    erg = self.classificator.classify_image(extracted_img, obj_id)
                    if erg[0] == 1:
                        enum = "unicorn"
                    elif erg[0] == 2:
                        enum = "cat"
                    elif erg[0] == 0:
                        enum = "other"
                    print("ID: ",obj_id," Class: ",enum)
                    self.classified_objects.add(obj_id)
                    #self.send_goal(0.19, y_pred_px * self.px_to_gantry_factor, seconds_until_grip)

                #cv2.imshow(f"Object {obj_id}", extracted_img)
                
                cv2.circle(frame_cropped, (cx, cy), 4, (0, 0, 255), -1)
                cv2.drawContours(frame_cropped, [contour], -1, (0, 255, 0), 2)
                cv2.putText(frame_cropped, 
                                  f'ID:{obj_id} ( {int(area)} px)', 
                                  (cx-20, cy-10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 240, 240), 2)
        
            
            cv2.imshow("Object Detection + Prediction", frame_cropped)

            key = cv2.waitKey(30)
            if key == 27 or key == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    action_client = TrackerClient()

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        print("\nTracker stopped by user.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':  
    main()
