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
iHighH = 255

iLowS = 0
iHighS =255

iLowV = 0
iHighV = 255

#We need to define this nothing function, since the sliders require a function to call if their values change
def nothing(x):
	pass

#Create trackbars in "Control"Window
cv2.createTrackbar("LowB","Control",0,255,nothing)
cv2.createTrackbar("HighB","Control",255,255,nothing)

cv2.createTrackbar("LowG","Control",0,255,nothing)
cv2.createTrackbar("HighG","Control",255,255,nothing)

cv2.createTrackbar("LowR","Control",0,255,nothing)
cv2.createTrackbar("HighR","Control",255,255,nothing)

# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	
	
	#Read om the values from the trackbars
	iLowH = cv2.getTrackbarPos("LowB","Control")
	iHighH = cv2.getTrackbarPos("HighB","Control")
	iLowS = cv2.getTrackbarPos("LowG","Control")
	iHighS = cv2.getTrackbarPos("HighG","Control")
	iLowV = cv2.getTrackbarPos("LowR","Control")
	iHighV = cv2.getTrackbarPos("HighR","Control")

	
	
	#Blur image and convert to the HSV color space
	image = cv2.GaussianBlur(image,(11,11),0)  #Apply Gassian Blur to 
	
	imgHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
	
	
	#cv2.imshow("HSV", imgHSV)

	#Color threshold the image
	imgthresh = cv2.inRange(imgHSV,np.array([iLowH, iLowS, iLowV]), np.array([iHighH, iHighS, iHighV]))
	
	
	###TODO - add dilate and erode here #####
	##see squares.py example in 'samples'
	#imgthresh = cv2.erode(imgthresh,None)
	#imgthresh = cv2.erode(imgthresh,None)
	imgthresh = cv2.dilate(imgthresh,None)
	#imgthresh = cv2.dilate(imgthresh,None)
	
	
	#Track the largest object
	_,contours,hierarchy = cv2.findContours(imgthresh.copy(),cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
	try:
		numobject = len(hierarchy[0])
		refArea = 0
		objectFound = False
		largestIndex =0
		largestx = 0
		largesty =0
		if numobject < 100: 
			for index in range(0,numobject):
				moment = cv2.moments(contours[index])
				area = int(moment['m00'])
				print(area)
			
				#if the area is less than 20 px by 20px then it is probably just noise
				#if the area is the same as the 3/2 of the image size, probably just a bad filter
				#we only want the object with the largest area so we save a reference area each
				#iteration and compare it to the area in the next iteration.
				if area>10 and area<100000 and area>refArea:
					x = int(moment['m10']) / area
					y = int(moment['m01']) / area
					objectFound = True
					refArea = area
					#save index of largest contour to use with drawContours
					largestIndex = index
					largestx = x
					largesty = y
				
				else:
					objectFound = False
			
				#if  objectFound == True:
				#	#putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2)
				#	#draw object location on screen
				#	print("x")
				#	print(x)
				#	print("y")
				#	print(y)
				#	cv2.circle(imgthresh,(x,y),15,(254,0,121),-1)
				#	#draw largest contour
				#	#drawContours(cameraFeed, contours, largestIndex, Scalar(0, 255, 255), 2)
		
			cv2.circle(image,(largestx,largesty),15,(254,255,255),-1)	
			cv2.imshow("Blurred",image)
			
		else:
			print('Too much noise')
			#putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2)
	except:
		print 'No object detected'
	
	
			
	
	#print numobject
	
	
	# show the thresholded image
	cv2.imshow("Frame", imgthresh)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
