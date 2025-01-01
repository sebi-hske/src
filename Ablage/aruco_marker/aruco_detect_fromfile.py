from __future__ import print_function
import sys # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
 
desired_aruco_dictionary = "DICT_4X4_50"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
  
def main():
    """
    Main method of the program.
    """
    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
      print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
      sys.exit(0)
    
    # Load the ArUco dictionary
    print("[INFO] detecting '{}' markers...".format(
      desired_aruco_dictionary))
    this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
    this_aruco_parameters = cv2.aruco.DetectorParameters()
    
    # Start the video 
    image = cv2.imread('/home/sebi/ros2_ws/src/Ablage/AMarkerID0.jpg')
    
    
     
    # Detect ArUco markers in the video image
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
      image, this_aruco_dictionary, parameters=this_aruco_parameters)
    # Check that at least one ArUco marker was detected
    if len(corners) > 0:
      # Flatten the ArUco IDs list
      ids = ids.flatten()
      # Loop over the detected ArUco corners
      for (marker_corner, marker_id) in zip(corners, ids):
    
        # Extract the marker corners
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners
        # Convert the (x,y) coordinate pairs to integers
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
        # Draw the bounding box of the ArUco detection
        cv2.line(image, top_left, top_right, (0, 255, 0), 2)
        cv2.line(image, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(image, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(image, bottom_left, top_left, (0, 255, 0), 2)
        # Calculate and draw the center of the ArUco marker
        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)
        # Draw the ArUco marker ID on the video image
        # The ID is always located at the top_left of the ArUco marker
        cv2.putText(image, str(marker_id), 
          (top_left[0], top_left[1] - 15),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5, (0, 255, 0), 2)
        
    
    # Display the resulting image
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.imshow('image',image)
          
    if cv2.waitKey(0) & 0xFF == ord('q'):
      cv2.destroyAllWindows()
      sys.exit()
  
  
  
   
print(__doc__)
main()