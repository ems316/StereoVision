# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
 
#Note this code was taken from 
#http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)
 
# allow the camera to warmup
time.sleep(0.1)
 
# grab an image from the camera
camera.capture(rawCapture, format="bgr")
image = rawCapture.array

#create a windo called "Control"
cv2.namedWindow("Control",cv2.WINDOW_AUTOSIZE)

iLowH = 0
iHighH = 179

iLowS = 0
iHighS =255

iLowV = 0
iHighV = 255

def nothing(x):
	pass



#Create trackbars in "Control"Window
cv2.createTrackbar("LowH","Control",0,179,nothing)
cv2.createTrackbar("HighH","Control",179,179,nothing)

cv2.createTrackbar("LowS","Control",0,255,nothing)
cv2.createTrackbar("HighS","Control",255,255,nothing)

cv2.createTrackbar("LowV","Control",0,255,nothing)
cv2.createTrackbar("HighV","Control",255,255,nothing)

iLowH = cv2.getTrackbarPos("LowH","Control")
iHighH = cv2.getTrackbarPos("HighH","Control")
iLowS = cv2.getTrackbarPos("LowS","Control")
iHighS = cv2.getTrackbarPos("HighS","Control")
iLowV = cv2.getTrackbarPos("LowV","Control")
iHighV = cv2.getTrackbarPos("HighV","Control")


imgHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
#cv2.imshow("HSV", imgHSV)


imgThresholded = cv2.inRange(image,np.array([iLowH, iLowS, iLowV]), np.array([iHighH, iHighS, iHighV]))

# display the image on screen and wait for a keypress
cv2.imshow("TresholdedImage", imgThresholded)
cv2.waitKey(0)