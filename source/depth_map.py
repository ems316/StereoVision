import numpy as np
import cv2
from matplotlib import pyplot as plt
import time

imgL = cv2.imread('scene_left.bmp',0)
imgR = cv2.imread('scene_right.bmp',0)
     
start = time.time()
stereo = cv2.StereoBM_create(numDisparities=32, blockSize=21)
disparity = stereo.compute(imgL,imgR)
print "Elapsed Time: ",time.time()-start

plt.imshow(disparity,'gray')
plt.show()