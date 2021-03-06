from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from matplotlib import pyplot as plt

#camera = PiCamera()
#camera.resolution = (640,480)
#camera.framerate = 32
#rawCapture = PiRGBArray(camera, size=(640,480))

#time.sleep(0.1)

#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #image = frame.array
cap2 = cv2.VideoCapture("udpsrc port=5003 ! application/x-rtp ! rtpjitterbuffer latency=0 ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! appsink sync =false")
cap1 = cv2.VideoCapture("fdsrc ! video/x-h264, width=640, height=480, framerate=10/1 ! h264parse ! omxh264dec ! videoconvert ! appsink sync=false")
while(True):
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities=112, SADWindowSize=15)
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    #disparity = stereo.compute(gray1, gray2)
    #plt.imshow(disparity, 'gray')
    #plt.show()
    

    cv2.imshow("Frame1", frame1)
    cv2.imshow("Frame2", frame2)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities=16, SADWindowSize=15)                                                                                                                                                    
#disparity = stereo.compute(frame1, frame)                                                                                                                                                                              
#plt.imshow(disparity, 'gray')
