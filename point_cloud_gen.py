import cv2 as cv
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
img = cv.imread('Sample_Data/Laser_Bottle/test1.jpg', cv.IMREAD_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"
img = cv.medianBlur(img,5)
ret,th1 = cv.threshold(img,200,255,cv.THRESH_BINARY)

def find_lowest_x_value(array):
    array = np.array(array)
    x_values = array[:,1]
    minx = np.min(x_values)
    
    return minx

def generate_z_values(base, obj):
    z = []
    for i in range(len(obj)):
        zval = obj[i][0] - base
        z.append(zval)
    z = np.array(z)
    xyz = np.concatenate((obj, z.reshape(-1, 1)), axis=1)
    return xyz
        

# Find contours in the thresholded image
contours, hierarchy = cv.findContours(th1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# Print the number of contours found
print("Number of contours found:", len(contours))
base_contours = []
obj_contours = []
wall_contours = []
# Iterate over each contour and print its coordinates
for i, contour in enumerate(contours):
    if(i == 0):
        base_contours.append(contour[:, 0, :])
    elif(i ==1):
        obj_contours.append(contour[:, 0, :])
    else:
        wall_contours.append(contour[:, 0, :])
    #print("Contour", i+1, ":", contour[:, 0, :])
minx = (find_lowest_x_value(base_contours))
obj_coords = generate_z_values(minx, obj_contours)
print(obj_coords)
plt.imshow(th1,'gray')

plt.show()