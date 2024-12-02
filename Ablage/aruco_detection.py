import cv2
import numpy as np

# Define the ArUco dictionary and parameters for marker detection
aruco_dict = cv2.aruco.Dictionary(cv2.aruco.DICT_4X4_50, 1)  # You can use different dictionary types
parameters = cv2.aruco.DetectorParameters()

# Camera calibration parameters (adjust these for your specific camera)
# These values are placeholders, you need to use your camera's calibration data
# (Camera Matrix and Distortion Coefficients)
camera_matrix = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=np.float32)  # Example values
dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # Assuming no distortion (adjust accordingly)

# GStreamer pipeline for capturing video (replace with your actual GStreamer pipeline)
gst_pipeline = "udpsrc address=10.42.0.206 port=5600 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"


# Initialize the GStreamer capture (OpenCV will handle the GStreamer pipeline)
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Unable to open video stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to fetch frame.")
        break

    # Convert the frame to grayscale (needed for ArUco detection)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Draw the markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose for each detected marker
        for i in range(len(ids)):
            # Get the rotation and translation vectors
            retval, rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, dist_coeffs)

            # Draw the axis of the detected marker
            cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

            # Calculate the distance to the marker (Euclidean distance)
            distance = np.linalg.norm(tvec)

            # Calculate the orientation of the marker
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            pitch, yaw, roll = cv2.decomposeProjectionMatrix(np.hstack((rotation_matrix, tvec)))[6]

            # Display the distance and orientation
            cv2.putText(frame, f"Distance: {distance:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Roll: {roll:.2f}",
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Show the frame with markers and pose
    cv2.imshow('ArUco Marker Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
