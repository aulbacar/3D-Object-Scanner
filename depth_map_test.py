import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
imgL = cv.imread('Sample_Data/Keys_Lit/test23.jpg', cv.IMREAD_GRAYSCALE)
imgR = cv.imread('Sample_Data/Keys_Lit/test0.jpg', cv.IMREAD_GRAYSCALE)
stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
#For hole 128 and 127 seems to give something useable
disparity = stereo.compute(imgL,imgR)
plt.imshow(disparity,'gray')
plt.show()
#print(disparity)