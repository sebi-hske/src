import cv2
import os
import logging

class ImageStorage:
    def __init__(self):

        self.CHESS_BOARD_DIM = (9, 6)

        #Verzeichnis gespeicherte Bilder
        self.image_dir_path = "~/kamera/bilder" #Anpassen
        
        #Kamera initialisierung
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Kamera konnte nicht geöffnet werden.")

        logging.basicConfig(level=logging.INFO)

        #Marker Erkennung initialisieren
        self.aruco_dict =  cv2.aruco.Dicitonary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def get_marker_offset(self):
        ret, frame = self.cap.read()
        if not ret:
            logging.error("Kein Bild von der Kamera erhalten.")
            return None

        cv2.imwrite(f"{self.image_dir_path}/image{self.n}.png", frame)
        print(f"Saved image number {self.n}")
        self.n += 1
        
        #Marker im Bild erkennen
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        #Marker erkannt
        if ids is not None:
            centers = [(c[0][0][0] + c[0][2][2]) / 2 for c in corners]
            avg_center = sum(centers) / len(centers)
            frame_center = frame.shape[1] / 2
            offset = avg_center - frame_center
            logging.info(f"Marker erkannt, Offset zur Mitte: {offset}")
            print(offset)
            return offset
        else:
            logging.warning("Keine Marker erkannt.")
            return None

    def release(self):
        self.cap.release()
 
