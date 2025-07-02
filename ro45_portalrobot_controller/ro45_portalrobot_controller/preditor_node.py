import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ro45_action_interfaces.action import Intercept


import cv2
import numpy as np
import time
import math

mm_per_pixel = 0.5
grip_x_px = 1300
grip_x_mm = grip_x_px * mm_per_pixel

class MotionPredictor:
    def __init__(self):
        self.position_history = {}  # {id: [(x_mm, y_mm, timestamp)]}
        self.prediction_given = set()

    def update(self, obj_id, x_mm, y_mm, timestamp, in_speed_zone):
        if obj_id not in self.position_history:
            self.position_history[obj_id] = []
        if in_speed_zone:
            self.position_history[obj_id].append((x_mm, y_mm, timestamp))
            self.position_history[obj_id] = [
                (x, y, t) for (x, y, t) in self.position_history[obj_id] if timestamp - t <= 10.0
            ]

    def compute_velocity(self, obj_id):
        data = self.position_history.get(obj_id, [])
        if len(data) < 2:
            return None
        x0, _, t0 = data[0]
        x1, _, t1 = data[-1]
        if t1 - t0 == 0:
            return None
        return (x1 - x0) / (t1 - t0)

    def predict_arrival(self, obj_id, current_time, target_x_mm):
        data = self.position_history.get(obj_id, [])
        if not data:
            return None, None, None, False
        x1, y1, _ = data[-1]
        vx = self.compute_velocity(obj_id)
        if vx is None or vx <= 0:
            return None, None, None, False
        dt = (target_x_mm - x1) / vx
        if dt < 0:
            return None, None, None, False
        return vx, current_time + dt, (target_x_mm, y1), True


class PredictorNode(Node):
    def __init__(self):
        super().__init__('predictor_node')
        self.action_client = ActionClient(self, Intercept, 'intercept_object')
        self.predictor = MotionPredictor()
        self.video_path = '/home/sebi/ros2_ws/RobotikProjektSoSe2025/Videos/WIN_20250522_13_44_48_Pro.mp4'

    def find_object_id(self, cx, cy):
        # Placeholder for object ID assignment logic
        return 1

    def detect_objects(self):
        cap = cv2.VideoCapture(0)
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
                
            now = time.time()
            
            # Process frame and find contours
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 100:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Convert to mm
                        x_mm = cx * mm_per_pixel
                        y_mm = cy * mm_per_pixel
                        
                        obj_id = self.find_object_id(cx, cy)
                        in_zone = False  # Placeholder for in_zone logic
                        self.predictor.update(obj_id, x_mm, y_mm, now, in_zone)

                        if not in_zone and obj_id not in self.predictor.prediction_given:
                            vx, arrival_time, (x_pred, y_pred), valid = self.predictor.predict_arrival(
                                obj_id, now, grip_x_mm
                            )

                            if valid:
                                goal_msg = Intercept.Goal()
                                goal_msg.time = float(arrival_time)
                                goal_msg.position_x = float(x_pred)
                                goal_msg.position_y = float(y_pred)
                                goal_msg.object_class = 2
                                
                                # Send goal without waiting
                                self.action_client.send_goal_async(goal_msg)
                                self.get_logger().info(f"Sent goal for object {obj_id}")
                                self.predictor.prediction_given.add(obj_id)

            # Show processed frame
            cv2.imshow('Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Cleanup
        cap.release()
        cv2.destroyAllWindows()

   


def main(args=None):
    rclpy.init(args=args)
    node = PredictorNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
