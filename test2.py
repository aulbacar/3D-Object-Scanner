import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
img = cv.imread('Sample_Data/Laser_Bottle/test4.jpg', cv.IMREAD_GRAYSCALE)
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
        arrx.append(1)
        arry.append(arr[i][1])
        arrz.append(arr[i][2])
    
    i += 1


i = 0

while(i < len(filter_arr)):
    filter_arr[i][0] = 1
    i += 1
#print(filter_arr)

i = 1
c = 0
length = len(filter_arr)
arrx2 = []
arry2 = []
arrz2 = []
while(i < 10):
    print(i)
    while(c < length):
        temp = filter_arr[c]
        #print(temp)
        temp[0] = i
        filter_arr.append(temp)
        arrx2.append(filter_arr[c][0])
        arry2.append(filter_arr[c][1])
        arrz2.append(filter_arr[c][2])
        c += 1
    c = 0
    i += 1

#print(filter_arr)


ax = plt.axes(projection = '3d')
ax.plot3D(arrx2, arry2, arrz2, 'gray')
plt.show()