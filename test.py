import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
img = cv.imread('Sample_Data/Laser_Bottle/test1.jpg', cv.IMREAD_GRAYSCALE)
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
            temp = [c, i, z]
            arr.append(temp)
        c += 1
    c = 0
    i += 1

i = 0
while(i < len(arr)):
    arr[i][2] = maxx - arr[i][0]
    i += 1
i = 0

filter_arr = []
while(i < len(arr)):
    if(arr[i][2] > 0 and arr[i][2] < 150):
        filter_arr.append(arr[i])
    
    i += 1


print(maxy)
print(maxx)

print(filter_arr)
