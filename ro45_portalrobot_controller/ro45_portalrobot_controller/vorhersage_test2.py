import cv2
import numpy as np
import time
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ro45_action_interfaces.action import Intercept
from ro45_portalrobot_controller.motion_predictor import MotionPredictor


    
class TrackerClient(Node):
    def __init__(self):
        super().__init__('action_client')
        self.action_client = ActionClient(self, Intercept, 'intercept_object')
        self.next_id = 1
        self.tracked_objects = []
        self.predictor = MotionPredictor()
        self.loop()  # Start the tracker loop

    def send_goal(self, x, y, time):
        self.get_logger().info(f"Sending goal")
        goal_msg = Intercept.Goal()
        goal_msg.position_x = float(x)
        goal_msg.position_y = float(y)
        goal_msg.time = time
        goal_msg.object_class = 1
    
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
        distance_threshold_px = 200
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

    def loop(self):
        mm_per_pixel = 0.5
        
        min_area = 8000
        max_area = 100000
        min_corners = 6

        # Cropping area
        crop_left = 92
        crop_right = 659
        crop_top = 500
        crop_bottom = 383

        grip_x_px = 1300
        grip_x_mm = grip_x_px * mm_per_pixel

        # Bereich zur Geschwindigkeitsmessung (Pixelwerte im Originalbild)
        speed_zone_left = 400
        speed_zone_right = 700
        speed_zone_top = 600
        speed_zone_bottom = 700
        video_path = "/home/sebi/ros2_ws/RobotikProjektSoSe2025/Videos/WIN_20250522_13_44_48_Pro.mp4"
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print("⚠️ Could not load video.")
            exit()

        time.sleep(2)
        cv2.namedWindow("Object Detection + Prediction", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Object Detection + Prediction", 960, 540)

        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("⚠️ Failed to grab frame")
                continue

            now = time.time()
            self.remove_stale_objects(now)

            frame_cropped = self.crop_image(frame, crop_left, crop_right, crop_top, crop_bottom)
            hsv = cv2.cvtColor(frame_cropped, cv2.COLOR_BGR2HSV)
            lower_bound = np.array([0, 0, 180])
            upper_bound = np.array([180, 60, 255])
            mask = cv2.inRange(hsv, lower_bound, upper_bound)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if not (min_area < area < max_area):
                    continue
                if self.count_corners(contour) < min_corners:
                    continue

                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"]) + crop_left
                cy = int(M["m01"] / M["m00"]) + crop_top

                if not (300 < cx < 800 and 600 < cy < 700):
                    continue

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

                        cv2.circle(frame, (x_pred_px, y_pred_px), 8, (255, 0, 0), 2)
                        cv2.putText(frame, f"Grab in {seconds_until_grip:.1f}s", (x_pred_px + 10, y_pred_px),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        print(f"Prediction: ID {obj_id} | Velocity = {vx:.2f} mm/s | ETA = {seconds_until_grip:.1f}s")
                        self.predictor.prediction_given.add(obj_id)
                    
                        self.send_goal(grip_x_mm, y_pred_px, seconds_until_grip)
                        

                
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(frame, f"ID {obj_id}", (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            frame_resized = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6)
            cv2.imshow("Object Detection + Prediction", frame_resized)

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
