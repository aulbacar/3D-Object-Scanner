import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
img = cv.imread('Sample_Data/Laser_Bottle/test20.jpg', cv.IMREAD_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"

ret,th1 = cv.threshold(img,200,255,cv.THRESH_BINARY)

plt.imshow(th1, "gray")
plt.show()


i = 0
c = 0
z = 0

arr = []
temp = []
maxy = 0
maxx = 0

while(i < len(th1)):
    while(c < len(th1[0])):
        if(th1[i][c] == 255):
            if(i > maxy):
                maxy = i
                maxx = c
            temp = [c, z, i]
            arr.append(temp)
        c += 1
    c = 0
    i += 1

i = 0
while(i < len(arr)):
    arr[i][1] = maxx - arr[i][0]
    #arr[i][0] = 1
    i += 1
i = 0

arrx = []
arry = []
arrz = []
filter_arr = []
while(i < len(arr)):
    if(arr[i][1] > 0 and arr[i][1] < 150):
        filter_arr.append(arr[i])
        arrx.append(arr[i][0])
        arry.append(arr[i][1])
        arrz.append(arr[i][2])
    
    i += 1

#ax = plt.axes(projection = '3d')
#ax.plot3D(filter_arr, 'gray')

#print(maxy)
#print(maxx)

#print(filter_arr)
unique_arrz = list(set(arrz))
#print(unique_arrz)



maxx_at_z = 0
minx_at_z = 480
i = 0
c = 0
minx_arr = []    
while(i < len(unique_arrz)):
    try_val = unique_arrz[i]
    while(c < len(filter_arr)):
        if(filter_arr[c][2] == try_val):
            #print(filter_arr[c])
            #if(filter_arr[c][0] > maxx_at_z):
                #maxx_at_z = filter_arr[c][0]
                #print("max ", maxx_at_z)
            if(filter_arr[c][0] < minx_at_z):
                minx_at_z = filter_arr[c][0]
                #print("min", minx_at_z)
                #print(filter_arr[c])
                minx_arr.append(filter_arr[c][0])
        c += 1
    c = 0
    minx_at_z = 480
    i += 1
#print(minx_arr)
#ax = plt.axes(projection = '3d')
##ax.plot3D(arrx, arry, arrz, 'gray')
#plt.show()

#print(minx_arr)
i = 0
c = 0
#print(len(minx_arr))
#print(len(unique_arrz))
straight_filter_arr = []
while(i < len(filter_arr)):
    while(c < len(unique_arrz)):
        if(filter_arr[i][2] == unique_arrz[c]):
            filter_arr[i][0] = filter_arr[i][0] - int(minx_arr[c]) + 1
            straight_filter_arr.append(filter_arr[i])
        c += 1
    c = 0
    i += 1
#print(straight_filter_arr)




#for i in th1:
    #for j in i:
        #if(th1[i][j] >= 200):
            #arr.append("j", j, "i", i)

#print(arr)            
#print(th1)
#print(ret)
#print(img)
#gray = cv.cvtColor(th1, cv.COLOR_BGR2GRAY)
#ret, thresh = cv.threshold(gray, 127, 255, 0)
#contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

#print(contours)
#plt.imshow(thresh)
#plt.show()