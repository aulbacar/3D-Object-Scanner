import cv2 as cv
import open3d as o3d
import numpy as np
import math
from matplotlib import pyplot as plt
from collections import Counter

img = cv.imread(f'Sample_Data/Laser_Bottle/test1.jpg', cv.IMREAD_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"

ret,th1 = cv.threshold(img,200,255,cv.THRESH_BINARY)


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

unique_arrz = list(set(arrz))

maxx_at_z = 0
minx_at_z = 480
i = 0
c = 0
minx_arr = []    
while(i < len(unique_arrz)):
    try_val = unique_arrz[i]
    while(c < len(filter_arr)):
        if(filter_arr[c][2] == try_val):
            if(filter_arr[c][0] < minx_at_z):
                minx_at_z = filter_arr[c][0]
                minx_arr.append(filter_arr[c][0])
        c += 1
    c = 0
    minx_at_z = 480
    i += 1

i = 0
c = 0
straight_filter_arr = []
while(i < len(filter_arr)):
    while(c < len(unique_arrz)):
        if(filter_arr[i][2] == unique_arrz[c]):
            filter_arr[i][0] = filter_arr[i][0] - int(minx_arr[c]) + 1
            straight_filter_arr.append(filter_arr[i])
        c += 1
    c = 0
    i += 1
    
arr = straight_filter_arr
x_values = [coord[0] for coord in arr]
y_values = [coord[1] for coord in arr]
x_counts = Counter(x_values)
y_counts = Counter(y_values)
min_num_x = 240
min_num_y = 130
most_common_x = [(x, count) for x, count in x_counts.items() if count >= min_num_x]
most_common_y = [(y, count) for y, count in y_counts.items() if count >= min_num_y]

x_min = min([x[0] for x in most_common_x])
x_max = max([x[0] for x in most_common_x])
y_min = min([y[0] for y in most_common_y])
y_max = max([y[0] for y in most_common_y])

highest_x_lowest_y = arr[0]
lowest_x_highest_y = arr[0]

# Loop through the coordinates and compare each coordinate's x and y values with the most common x and y values
for coord in arr:
    x, y, z = coord
    if x == x_max and (y > lowest_x_highest_y[1]):
        lowest_x_highest_y = coord
    elif x == x_min and (y < highest_x_lowest_y[1]):
        highest_x_lowest_y = coord
        
mid_x = int((highest_x_lowest_y[0] + lowest_x_highest_y[0]) / 2)
mid_y = int((highest_x_lowest_y[1] + lowest_x_highest_y[1]) / 2)
midpoint = (mid_x, mid_y)
# print(midpoint)
    
for i in range(len(arr)):
    x, y, z = arr[i]
    x = x - midpoint[0]
    y = y - midpoint[1]
    arr[i] = [x, y, z]

##################################################################################################################
x_axis = [(i, 0, 0) for i in range(0,200)]
y_axis = [(0, i, 0) for i in range(0,300)]
z_axis = [(0, 0, i) for i in range(0,600)]

prev_arr = straight_filter_arr
final_arr = np.array(arr)
p = 2 * math.pi * 20 / 180
for i in range(len(final_arr)):
    x = final_arr[i, 0] * np.sin(p) + final_arr[i, 1] * np.sin(p)
    y = final_arr[i, 0] * np.sin(p) - final_arr[i, 1] * np.sin(p)
    z = final_arr[i, 2]
    final_arr[i] = [x, y, z]
    
pc_norotate = np.array(prev_arr + x_axis + y_axis + z_axis)
pc_rotate = final_arr
np.savetxt(f'rotate_tests/no_rotate.xyz', pc_norotate, delimiter=' ')
np.savetxt(f'rotate_tests/rotate.xyz', pc_rotate, delimiter=' ')

input_path1= f'rotate_tests/no_rotate.xyz'
point_cloud1= np.loadtxt(input_path1,skiprows=0)

input_path2= f'rotate_tests/rotate.xyz'
point_cloud2= np.loadtxt(input_path2,skiprows=0)

pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(point_cloud1[:,:3])

pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(point_cloud2[:,:3])
o3d.visualization.draw_geometries([pcd1, pcd2])