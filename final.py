import cv2 as cv
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt
import pyvista as pv
import math


image = cv.imread('Sample_Data/box/cal.jpg')
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(gray, 127, 255, 0)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
contours = np.vstack(contours).squeeze()
miny = 480


i = 0
while(i < len(contours)):
    if(contours[i][1] < miny):
        miny = contours[i][1]
    i += 1
print(miny)

ct = 0
combined_pc = np.empty((0, 3))
angle_offset = .05
x_offset = 0
y_offset = 0
for img_no in range(2,124):
    img = cv.imread(f'Sample_Data/box/test{img_no}.jpg', cv.IMREAD_GRAYSCALE)
    assert img is not None, "file could not be read, check with os.path.exists()"

    ret,th1 = cv.threshold(img,210,255,cv.THRESH_BINARY)


    i = 0
    c = 0
    z = 0

    arr = []
    temp = []
    maxx = 0
    max = 0

    i = 0
    c = 0
    while(i < len(th1)):
        while(c < len(th1[0])):
           if(th1[i][c] == 255):
                if(i > max):
                    max = 640 - c#(c + 200)
                    scale = int(max * .2)
                    maxx = 640 - (c - (scale))
                    maxy = i
           c += 1
        c = 0
        i += 1
    print(maxx)


    i = 0
    c = 0
    while(i < len(th1)):
        while(c < len(th1[0])):
            if(th1[i][c] == 255):

                if(i < 440 and i > miny): #110 for bottle
                    temp = [0, maxx-c, i]
                    arr.append(temp)
            
            

            c += 1
        
        c = 0
        i += 1

    i = 0
    avg = 0



    pc = np.array(arr)

  

    rotation_axis = np.array([0, 0, 1])  # rotate around the y-axis
    rotation_angle = img_no * angle_offset  # adjust the rotation angle as desired
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis*rotation_angle)
    
    pc[:, :3] = np.matmul(pc[:, :3], rotation_matrix)
    combined_pc = np.vstack((combined_pc, pc))
    ct +=1
    print(ct)

points = combined_pc
cloud = pv.PolyData(points)
cloud.plot(point_size=2)
surf = cloud.delaunay_2d()
surf.plot(show_edges=True)
surf.save('mesh.stl')
