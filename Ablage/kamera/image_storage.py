import cv2
import os
import time

class ImageStorage:
    def __init__(self):
        self.CHESS_BOARD_DIM = (9, 6) #size of a chess board

        #initialize the variables
        self.n = 0  # Number of images taken
        self.max_images = 10  # Maximum number of images to capture
        self.image_dir_path = "/home/ubuntu/kamera/images"

        #Check if the directory exists, create it if it doesn't
        CHECK_DIR = os.path.isdir(self.image_dir_path)
        if not CHECK_DIR:
            os.makedirs(self.image_dir_path)
            print("Directory created")
        else:
            print("Directory exists")

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)  #criteria for corner refinement

        self.cap = cv2.VideoCapture(0) #default camera access

    def save_image(self):
        ret, frame = self.cap.read() #capture frames from the camera
        if not ret:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convert images to a gray scale
        ret, corners = cv2.findChessboardCorners(gray, self.CHESS_BOARD_DIM) #find the corners

        if ret:
            #for better accuracy, image corners should be refined
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            if corners_refined is not None:
                #save the image with the given corners
                cv2.imwrite(f"{self.image_dir_path}/image{self.n}.png", frame)
                print(f"Saved image number {self.n}")
                self.n += 1 #increment the image amount
            else:
                print("Corner refinement failed.")
        else:
            print("No corners detected.")

    def start_capture(self): #capture max number of images (10)
        while self.n < self.max_images:
            self.save_image()
            time.sleep(5)

        self.cap.release()

#Create an instance of ImageStorage and start capturing images
storage = ImageStorage()
storage.start_capture()
