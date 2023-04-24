import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from matplotlib import cm
def toTuple(contour, x_arr, y_arr, inc):
    i = 0
    while(i < len(contour)):
        testString = str(contour[i][0])
        testString = testString.replace('[[', '')
        testString = testString.replace(']]', '')
        string_seperate = testString.split()
        x_arr.append(int(string_seperate[0]))
        y_arr.append(int(string_seperate[1]))
        
        ang_arr.append(inc * 15)
        i += 1

   

i = 0
x_arr = []
y_arr = []
#z_arr = []
ang_arr = []
contour_array = []
while(i < 25):
    image = cv.imread('Sample_Data/Laser_Bottle/test' + str(i) + '.jpg')
    #cv.waitKey(0)
    assert image is not None, "file could not be read, check with os.path.exists()"
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 127, 255, 0)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cv.imshow('Canny Edges After Contouring', thresh)
    cv.imshow('Contours', image)
    #print(contours)
    toTuple(contours, x_arr, y_arr, i)
    #print(x_arr)
    #print(y_arr)
    #print(t2)
    #c = 0
    #while(c < len(contours)):
        #print(contours[c][0])
        #c += 1
    cv.waitKey(0)
    i += 1

#print(ang_arr[1160])
#print(ang_arr)
#fig = plt.figure()
#ax = fig.add_subplot(projection='3d')
#ax.plot(x_arr, y_arr)
#ROTATE AROUND Z-AXIS
i = 0
#while(i < len(x_arr)):
    #fig = plt.figure()
    #ax = fig.add_subplot(projection='3d')
    #plt.plot(x_arr[i], y_arr[i])
    #ax.scatter(x_arr[i], y_arr[i], ang_arr[i])
    #plt.draw()
    #plt.pause(.2)
   # i += 1
#for coord in x_arr:
    #ax.plot3D(x_arr[coord], ang_arr[coord], y_arr[coord])
    #plt.show()
#ax.plot3D(x_arr,ang_arr, y_arr)
#plt.scatter(x_arr, y_arr, ang_arr)
#plt.show()
#print(y_arr)
#print(len(x_arr))
#print(len(y_arr))
#print(len(ang_arr))

#plt.plot(x_arr, y_arr)
#plt.show()