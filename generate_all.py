import cv2 as cv
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
for img_no in range(1,24):
    img = cv.imread(f'Sample_Data/Laser_Bottle/test{img_no}.jpg', cv.IMREAD_GRAYSCALE)
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

    points = np.array(filter_arr)
    np.savetxt(f'point_clouds/pc{img_no}.xyz', points, delimiter=' ')

    input_path= f"point_clouds/pc{img_no}.xyz"
    point_cloud= np.loadtxt(input_path,skiprows=0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
# o3d.visualization.draw_geometries([pcd])