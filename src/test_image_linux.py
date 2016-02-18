import numpy as np
import cv2

#cap.set(CV_CAP_PROP_FRAME_WIDTH, 640)
#cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480)
cap = cv2.VideoCapture("/dev/stdin")#"udpsrc port=5001 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! fpsdisplaysink sync=false text-overlay=false")
cap.set(3, 640)
cap.set(4, 480)
while(True):
    ret, frame = cap.read()

    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    while frame is None:
        ret, frame = cap.read()
    

    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
