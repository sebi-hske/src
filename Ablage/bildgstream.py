import cv2

gstreamer_str = "udpsrc address=10.42.0.206 port=5600 ! application/x-rtp, encoding-name=H264 ! rtph264depay ! avdec_h264 ! autovideosink"
  # Start the video stream
cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)

if cap.isOpened() is not True:
        print("no video")
        #exit()

while(True):
    status, photo = cap.read()
    if not status:
        print("Error: Failed to capture frame.")

    #cv2.imshow("robbi", photo)
    if cv2.waitKey(10) == 13:
        break
        
cap.release()
cv2.destroyAllWindows()