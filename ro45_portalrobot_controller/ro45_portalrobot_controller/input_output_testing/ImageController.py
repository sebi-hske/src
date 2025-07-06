#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np

class ImageController:
    
    @staticmethod
    def crop(image, crop_left, crop_right, crop_top, crop_bottom):
        height, width = image.shape[:2]
        start_x = crop_left
        end_x = width - crop_right
        start_y = crop_top
        end_y = height - crop_bottom
        if start_x < 0 or start_y < 0 or end_x > width or end_y > height:
            raise ValueError("Crop-Werte führen zu ungültiger Bildgröße.")
        if start_x >= end_x or start_y >= end_y:
            raise ValueError("Crop-Werte sind zu groß für das Bild.")
        return image[start_y:end_y, start_x:end_x]
    
    @staticmethod
    def crop_bounding_rect(image, src_pts):
        x_coords = src_pts[:, 0]
        y_coords = src_pts[:, 1]
        x_min = int(np.min(x_coords))
        x_max = int(np.max(x_coords))
        y_min = int(np.min(y_coords))
        y_max = int(np.max(y_coords))
        cropped_image = image[y_min:y_max, x_min:x_max]
        return cropped_image, x_min, y_min
    
    @staticmethod
    def rectify_cropped_image(cropped_image, src_pts, x_offset, y_offset):
        adjusted_pts = src_pts.copy()
        adjusted_pts[:, 0] -= x_offset
        adjusted_pts[:, 1] -= y_offset

        width_top = np.linalg.norm(adjusted_pts[1] - adjusted_pts[0])
        width_bottom = np.linalg.norm(adjusted_pts[2] - adjusted_pts[3])
        max_width = int(max(width_top, width_bottom))

        height_left = np.linalg.norm(adjusted_pts[3] - adjusted_pts[0])
        height_right = np.linalg.norm(adjusted_pts[2] - adjusted_pts[1])
        max_height = int(max(height_left, height_right))

        dst_pts = np.array([
            [0, 0],
            [max_width - 1, 0],
            [max_width - 1, max_height - 1],
            [0, max_height - 1]
        ], dtype="float32")

        H, _ = cv2.findHomography(adjusted_pts, dst_pts)
        warped = cv2.warpPerspective(cropped_image, H, (max_width, max_height))
        return warped