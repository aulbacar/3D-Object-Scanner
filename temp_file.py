import cv2 as cv
import open3d as o3d
import numpy as np
import math
from matplotlib import pyplot as plt
from collections import Counter
theta = 0.8726646
for img_no in range(2,125):
    img = cv.imread(f'Sample_Data/Object/test{img_no}.jpg', cv.IMREAD_GRAYSCALE)
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
    # print(arr)
    x_values = [coord[0] for coord in arr]
    y_values = [coord[1] for coord in arr]
    x_counts = Counter(x_values)
    y_counts = Counter(y_values)
    min_num_x = 240
    min_num_y = 130
    most_common_x = [(x, count) for x, count in x_counts.items() if count >= min_num_x]
    most_common_y = [(y, count) for y, count in y_counts.items() if count >= min_num_y]
    xval = [x[1] for x in most_common_x]
    yval = [y[1] for y in most_common_y]
    x_avg = 0
    for x in xval:
        x_avg += x
    x_avg = x_avg/(len(xval))

    y_avg = 0
    for y in yval:
        y_avg += y
    y_avg = y_avg/(len(yval))

    min_num_x = int(x_avg) - 20
    min_num_y = int(y_avg) - 20
    # print("Min x: ", min_num_x)
    # print("Min y: ", min_num_y)
    most_common_x = [(x, count) for x, count in x_counts.items() if count >= min_num_x]
    most_common_y = [(y, count) for y, count in y_counts.items() if count >= min_num_y]
    # print(most_common_x)
    # print(most_common_y)
    x_min = min([x[0] for x in most_common_x])
    x_max = max([x[0] for x in most_common_x])
    y_min = min([y[0] for y in most_common_y])
    y_max = max([y[0] for y in most_common_y])
    # print(x_min)
    # print(x_max)
    # print(y_min)
    # print(y_max)

    highest_x_lowest_y = arr[0]
    lowest_x_highest_y = arr[0]
    # Loop through the coordinates and compare each coordinate's x and y values with the most common x and y values
    for coord in arr:
        x, y, z = coord
        if x == x_max and (y <= y_min and y >= y_min-5):
            highest_x_lowest_y = coord
        if x == x_min and (y >= y_max):
            lowest_x_highest_y = coord
            
    # print("high x, low y: ", highest_x_lowest_y)
    # print("low x, high y: ",lowest_x_highest_y)
            
    mid_x = int((highest_x_lowest_y[0] + lowest_x_highest_y[0]) / 2)
    mid_y = int((highest_x_lowest_y[1] + lowest_x_highest_y[1]) / 2)
    midpoint = (mid_x, mid_y)
    # print(midpoint)
        
    # for i in range(len(arr)):
    #     x, y, z = arr[i]
    #     x = x - midpoint[0]
    #     y = y - midpoint[1]
    #     arr[i] = [x, y, z]

    ##################################################################################################################
    x_axis = [(i, 0, 0) for i in range(0,200)]
    y_axis = [(0, i, 0) for i in range(0,300)]
    z_axis = [(0, 0, i) for i in range(0,600)]

    pre_pc = np.array(arr)
    axes = np.array(x_axis + y_axis + z_axis)

    np.savetxt(f'rotate_tests/rotate{img_no}.xyz', pre_pc, delimiter=' ')
    np.savetxt(f'rotate_tests/axes.xyz', axes, delimiter=' ')

    input_path1= f'rotate_tests/axes.xyz'
    point_cloud1= np.loadtxt(input_path1,skiprows=0)

    input_path2= f'rotate_tests/rotate{img_no}.xyz'
    point_cloud2= np.loadtxt(input_path2,skiprows=0)
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1[:,:3])

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(point_cloud2[:,:3])
    # if(img_no != 1):
    theta += 0.1396263  # 45 degrees
    rot_z = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])

    # apply the rotation
    pcd2.rotate(rot_z)

    rotated_pc = np.asarray(pcd2.points)
    print(img_no)
    # Save the rotated point cloud to a file
    output_path = f"transformed/rotated_point_cloud{img_no}.xyz"
    np.savetxt(output_path, rotated_pc, delimiter=" ")

    # o3d.visualization.draw_geometries([pcd1, pcd2])