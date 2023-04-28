import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from matplotlib import cm

x_arr = []
y_arr = []
ang_arr = []
contour_array = []


image = cv.imread('Sample_Data/Laser_Bottle/test1.jpg')

assert image is not None, "file could not be read, check with os.path.exists()"
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(gray, 127, 255, 0)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

print(contours)
plt.imshow(thresh)
plt.show()