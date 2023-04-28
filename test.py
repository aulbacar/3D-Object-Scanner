import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
img = cv.imread('Sample_Data/Laser_Bottle/test1.jpg', cv.IMREAD_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"
#img = cv.medianBlur(img,5)
ret,th1 = cv.threshold(img,200,255,cv.THRESH_BINARY)

plt.imshow(th1, "gray")
plt.show()

#print(th1[448][308]) #[y coord][x coord]
#print(len(th1))
#print(len(th1[0]))

i = 0
c = 0
z = 0
#i = len(th1)
#c = (len(th1[0]))
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
            #arr.append(th1[i][c])
            arr.append(temp)
        c += 1
    c = 0
    i += 1
#print(arr)

i = 0
while(i < len(arr)):
    #print(arr[i][0])
    arr[i][2] = maxx - arr[i][0]
    i += 1

print(maxy)
print(maxx)
print(arr)





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