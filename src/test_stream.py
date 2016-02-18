import numpy as np
import cv2

cap = cv2.VideoCapture()
cap.open("udpsrc port=5003 ! application/x-rtp ! rtpjitterbuffer latency=0 ! rtph264depay ! ffvdec_h264 ! autoconvert ! appsink")
#cap.open("./inout")

while(cap.isOpened()==False):
	cap.open("udpsrc port=5003 ! application/x-rtp ! rtpjitterbuffer latency=0 ! rtph264depay ! autoconvert ! ffvdec_h264 ! appsink")
	cap.open("udpsrc port=5003 ! application/x-rtp ! rtpjitterbuffer latency=0 ! rtph264depay ! ffvdec_h264 ")
	cap.open("udpsrc port=5003 ! application/x-rtp ! rtpjitterbuffer latency=0 ! rtph264depay ! ffvdec_h264 ")
	cap.open("udpsrc port=5003  ! gdpdepay ! rtph264depay ! video/x-h264, width=1280, height=720, format=YUY2, framerate=49/1 ! ffdec_h264 ! autoconvert ! appsink sync=false")
	print cap.isOpened()
cap.set(3,640)
cap.set(4,480)



while(True):

	ret, frame = cap.read()

	#while frame is None:
	#	ret, frame = cap.read()

	if ret:
		cv2.imshow('0', frame)
		junk = cv2.waitKey(10)	

#cv2.imshow('frame', frame)

	#if cv2.waitKey(1) & 0xFF == ord('q'):
	#	break
#cap.release()
#cv2.destroyAllWindows()
