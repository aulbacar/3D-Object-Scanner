import cv2 as cv
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt
for img_no in range(1,125):
    img = cv.imread(f'Sample_Data/box/test{img_no}.jpg', cv.IMREAD_GRAYSCALE)
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

    points = np.array(straight_filter_arr)
    np.savetxt(f'shifted_clouds/pc{img_no}.xyz', points, delimiter=' ')

    input_path= f'shifted_clouds/pc{img_no}.xyz'
    point_cloud= np.loadtxt(input_path,skiprows=0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
#o3d.visualization.draw_geometries([pcd])