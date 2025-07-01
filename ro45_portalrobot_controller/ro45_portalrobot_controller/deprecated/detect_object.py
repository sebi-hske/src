import cv2
import numpy as np
from pathlib import Path

class WhiteObjectDetector:
    def __init__(self):
        self.window_name = 'White Object Detection (Size and Corner Filtered)'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.current_id = 0
        self.min_area = 8000  # Minimum area threshold
        self.max_area = 100000  # Maximum area threshold
        self.min_corners = 6

    def count_corners(self, contour):
        # Approximate contour to reduce number of points
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        return len(approx)

    def draw_coordinate_system(self, image, origin=(5, 5), size=40):
        # Draw X axis with arrow (right)
        cv2.arrowedLine(image, 
                        origin, 
                        (origin[0] + size, origin[1]), 
                        (0, 0, 255), 2, tipLength=0.3)
        
        # Draw Y axis with arrow (down)
        cv2.arrowedLine(image, 
                        origin,
                        (origin[0], origin[1] + size), 
                        (0, 0, 255), 2, tipLength=0.3)
        
        # Add axis labels
        cv2.putText(image, "x", 
                    (origin[0] + size + 5, origin[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        cv2.putText(image, "y",
                    (origin[0] - 5, origin[1] + size + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    def detect_objects(self, image_path):
        frame = cv2.imread(str(image_path))
        if frame is None:
            print(f'Failed to load image: {image_path}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, 
                                     cv2.CHAIN_APPROX_SIMPLE)

        display = frame.copy()
        
        # Draw coordinate system in top left
        self.draw_coordinate_system(display)

        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:  # Size filter
                corners = self.count_corners(contour)
                if corners > self.min_corners:  # Corner filter
                    self.current_id += 1
                    M = cv2.moments(contour)
                    
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw object info
                        cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.drawContours(display, [contour], -1, (0, 255, 0), 2)
                        cv2.putText(display, 
                                  f'ID:{self.current_id} ({corners} corners, {int(area)} px)', 
                                  (cx-20, cy-10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        cv2.putText(display, f'({cx},{cy})', 
                                  (cx-20, cy+20),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow(self.window_name, display)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main():
    detector = WhiteObjectDetector()
    image_path = input("Enter image path: ")
    detector.detect_objects(Path(image_path))

if __name__ == '__main__':
    main()
