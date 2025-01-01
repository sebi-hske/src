import cv2 as cv
import sys

cam = cv.VideoCapture(0)
if not cam:
    sys.exit('Could not open camera.')
ret_val, img = cam.read()
h, w = img.shape[:2]

img = img[h//2:, :] # Die obere Hälfte wegschneiden
img = cv.cvtColor(img, cv.COLOR_BGR2RGB)

if ret_val:
    cv.imwrite('Bild1.jpg', img)
else:
    sys.exit('Could not read from the camera.')
