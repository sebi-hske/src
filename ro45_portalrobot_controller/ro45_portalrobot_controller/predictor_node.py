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
        self.get_logger().info('Starting video analysis...')
        try:
            self.start_video_analysis()
        except Exception as e:
            self.get_logger().error(f'Failed to start video analysis: {str(e)}')
            return
    
    def send_intercept_goal(self, x, y, time):
        self.get_logger().info(f"Sending intercept goal to position: x={x}, y={y}, time={time}")
        goal_msg = Intercept.Goal()
        goal_msg.position_x = x
        goal_msg.position_y = y
        goal_msg.time = time
        goal_msg.object_class = 0    
    
        if not self.action_client.wait_for_server(5.0):
            self.get_logger().error("Action server not available. Cannot send goal.")
            return False
    
        # Send goal and get future
        goal_future = self.action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        try:
            goal_handle = goal_future.result(timeout=5.0)
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False
                
            # Get result future
            result_future = goal_handle.get_result_async()
            
            # Wait for result with timeout
            result = result_future.result(timeout=10.0)
            self.get_logger().info('Move completed successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Move failed with error: {str(e)}')
            return False

    def start_video_analysis(self):
        self.get_logger().info('Received prediction request.')

        cap = cv2.VideoCapture('/home/sebi/ros2_ws/RobotikProjektSoSe2025/Videos/WIN_20250522_13_44_48_Pro.mp4')
        if not cap.isOpened():
            self.get_logger().error('Failed to open video file.')
            return

        crop_left = 92
        crop_right = 659
        crop_top = 500
        crop_bottom = 383
        min_area = 8000
        max_area = 100000
        min_corners = 6
        speed_zone_left = 400
        speed_zone_right = 700
        speed_zone_top = 600
        speed_zone_bottom = 700

        def crop_image(image):
            height, width = image.shape[:2]
            return image[crop_top:height - crop_bottom, crop_left:width - crop_right]

        def count_corners(contour):
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            return len(approx)

        obj_id = 1
        predicted = False

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            now = time.time()
            cropped = crop_image(frame)
            hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
            lower = np.array([0, 0, 180])
            upper = np.array([180, 60, 255])
            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if not (min_area < area < max_area):
                    continue
                if count_corners(contour) < min_corners:
                    continue

                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"]) + crop_left
                cy = int(M["m01"] / M["m00"]) + crop_top

                in_zone = (speed_zone_left <= cx <= speed_zone_right and
                           speed_zone_top <= cy <= speed_zone_bottom)

                x_mm = cx * mm_per_pixel
                y_mm = cy * mm_per_pixel
                self.predictor.update(obj_id, x_mm, y_mm, now, in_zone)

                if not in_zone and obj_id not in self.predictor.prediction_given:
                    vx, arrival_time, (x_pred, y_pred), valid = self.predictor.predict_arrival(
                        obj_id, now, grip_x_mm
                    )
                    time_until_grip = arrival_time - now
                    self.send_intercept_goal(float(x_pred) if valid else 0.0,
                                            float(y_pred) if valid else 0.0,
                                            float(time_until_grip) if valid else 0.0)
                    cap.release()
                    
                    return

        cap.release()
        


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PredictorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()