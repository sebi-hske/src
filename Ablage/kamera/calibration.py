import cv2
import numpy as np
import os

class Calibration:
    def __init__(self, image_dir_path="/home/ubuntu/kamera/images", calib_data_path="/home/ubuntu/kamera/calib_data"):
        self.CHESS_BOARD_DIM = (9, 6) #the chessboard size
        self.SQUARE_SIZE = 2.1 #the size of each square
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #termination criteria
        self.image_dir_path = image_dir_path # path to the directory containing images
        self.calib_data_path = calib_data_path #path to save calibration data
        self.n =0

    def perform_calibration(self):
        CHECK_DIR = os.path.isdir(self.calib_data_path)  # Check if the calibration data directory exists, if not, create it

        if not CHECK_DIR:
            os.makedirs(self.calib_data_path)
            print("Directory created")
        else:
            print("Directory exists")

        obj_3d = np.zeros((self.CHESS_BOARD_DIM[0]*self.CHESS_BOARD_DIM[1], 3), np.float32) #initialize 3d object points
        obj_3d[:, :2] = np.mgrid[0:self.CHESS_BOARD_DIM[0], 0:self.CHESS_BOARD_DIM[1]].T.reshape(-1, 2)
        obj_3d *= self.SQUARE_SIZE

        #store 3D object points and 2D image points from chessboard corners
        obj_points_3d = []
        
        img_points_2d = []

        files = os.listdir(self.image_dir_path)#iterate the images
        for file in files:
            imagePath = os.path.join(self.image_dir_path, file)

            image = cv2.imread(imagePath)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert to a gray scale
            ret, corners = cv2.findChessboardCorners(gray, self.CHESS_BOARD_DIM, None)#find chessboard corners
            if ret:
                obj_points_3d.append(obj_3d)
                corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), self.criteria)
                img_points_2d.append(corners2)

                img = cv2.drawChessboardCorners(image, self.CHESS_BOARD_DIM, corners2, ret)#draw chessboard corners
                #cv2.imshow('Corners', img)
                cv2.imwrite(f"{self.image_dir_path}/image_calibration{self.n}.png", img)
                self.n+=1
                cv2.waitKey(500)
        
        cv2.destroyAllWindows()#close the windows


        #calculate camera calibration using object and image points
        ret, camera_matrix, distortion_coeff, rvecs, tvecs = cv2.calibrateCamera(
            obj_points_3d, img_points_2d, gray.shape[::-1], None, None
        )
        print("Calibrated")


          # Save calibration data to a .npz file
        np.savez(
            f"{self.calib_data_path}/calibration.npz",
            camera_matrix=camera_matrix,
            distortion_coefficients=distortion_coeff,
            rvecs=rvecs,
            tvecs=tvecs,
        )
        # verification check
        data = np.load(f"{self.calib_data_path}/calibration.npz")
        cam_matrix = data["camera_matrix"]
        dist_coef = data["distortion_coefficients"]
        r_vector = data["rvecs"]# mit distanz zu checken
        t_vector = data["tvecs"]

        print("Loaded calibration data successfully")

# Usage:
calibration = Calibration()
calibration.perform_calibration()
