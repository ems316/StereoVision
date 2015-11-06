import numpy as np
import cv2

cap = cv2.VideoCapture("udpsrc port=5003 ! application/x-rtp ! rtpjitterbuffer latency=0 ! rtph264depay ! avdec_h264 ! autovideosink sync=false")

cap.set(3,640)
cap.set(4,480)

while(True):

	ret, frame = cap.read()

	while frame is None:
		ret, frame = cap.read()

	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()
