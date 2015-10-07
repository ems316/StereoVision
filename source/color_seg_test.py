# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

#Create a window to control the filtering parameters
cv2.namedWindow("Control",cv2.WINDOW_AUTOSIZE)

#Define the filter parameters
iLowH = 0
iHighH = 179

iLowS = 0
iHighS =255

iLowV = 0
iHighV = 255

#We need to define this nothing function, since the sliders require a function to call if their values change
def nothing(x):
	pass

#Create trackbars in "Control"Window
cv2.createTrackbar("LowH","Control",0,255,nothing)
cv2.createTrackbar("HighH","Control",255,255,nothing)

cv2.createTrackbar("LowS","Control",0,255,nothing)
cv2.createTrackbar("HighS","Control",255,255,nothing)

cv2.createTrackbar("LowV","Control",0,255,nothing)
cv2.createTrackbar("HighV","Control",255,255,nothing)

# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	
	#Read om the values from the trackbars
	iLowH = cv2.getTrackbarPos("LowH","Control")
	iHighH = cv2.getTrackbarPos("HighH","Control")
	iLowS = cv2.getTrackbarPos("LowS","Control")
	iHighS = cv2.getTrackbarPos("HighS","Control")
	iLowV = cv2.getTrackbarPos("LowV","Control")
	iHighV = cv2.getTrackbarPos("HighV","Control")

	#Convert to the HSV color space
	#imgHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
	#cv2.imshow("HSV", imgHSV)

	#Color Segment the image
	imgThresholded = cv2.inRange(image,np.array([iLowH, iLowS, iLowV]), np.array([iHighH, iHighS, iHighV]))
 
	# show the thresholded image
	cv2.imshow("Frame", imgThresholded)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break