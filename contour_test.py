import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
def toTuple(contour, x_arr, y_arr):
    i = 0
    while(i < len(contour)):
        testString = str(contour[i][0])
        testString = testString.replace('[[', '')
        testString = testString.replace(']]', '')
        string_seperate = testString.split()
        x_arr.append(int(string_seperate[0]))
        y_arr.append(int(string_seperate[1]))
        i += 1

   

i = 0
x_arr = []
y_arr = []
z_arr = []
contour_array = []
while(i < 1):
    image = cv.imread('Sample_Data/Keys_Lit/test' + str(i) + '.jpg')
    #cv.waitKey(0)
    assert image is not None, "file could not be read, check with os.path.exists()"
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 127, 255, 0)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cv.imshow('Canny Edges After Contouring', thresh)
    #print(contours)
    toTuple(contours, x_arr, y_arr)
    #print(x_arr)
    #print(y_arr)
    #print(t2)
    #c = 0
    #while(c < len(contours)):
        #print(contours[c][0])
        #c += 1
    cv.waitKey(0)
    i += 1


