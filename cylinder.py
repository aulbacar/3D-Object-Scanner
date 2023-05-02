import cv2 as cv
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt
import pyvista as pv
import math

ct = 0
combined_pc = np.empty((0, 3))
angle_offset = .05
x_offset = 0
y_offset = 0
for img_no in range(2,124):
    img = cv.imread(f'Sample_Data/bottle/test{img_no}.jpg', cv.IMREAD_GRAYSCALE)
    assert img is not None, "file could not be read, check with os.path.exists()"

    ret,th1 = cv.threshold(img,210,255,cv.THRESH_BINARY)


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
            
                #temp = [c, z, i]
                #temp = [maxx, z, i]
                #arr.append(temp)
                #temp = [0, maxx - c, i]
                #arr.append(temp)
            #arr[i][1] = maxx - arr[i][0]
            temp = [0, maxx - c, i]
            arr.append(temp)
            #print(maxx-c)
            #print(temp)
            #print(maxx-c)
            if((i) < 100 and (i) > 440):
                print("added")
                #temp = [0, maxx - c, i]
                #arr.append(temp)
            c += 1
        
        c = 0
        i += 1

    i = 0
    avg = 0
    #while(i < len(arr)):
        #arr[i][1] = maxx - arr[i][0]
        #arr[i][0] = 150
        #avg += arr[i][0]
        
        #i += 1
    #avg = avg / i
    #print(i)
    #i = 0

    

    #print(avg)

    i = 0
    #while(i < len(arr)):
       #arr[i][0] = avg
       #i += 1  
        
    #print(arr[0])
        

    #while(i < len(arr)):
        #arr[i][0] = 1
        #arr[i][0] = 1
        #i += 1
    #i = 0

    #arrx = []
    #arry = []
    #arrz = []
    #filter_arr = []
    #while(i < len(arr)):
        #if(arr[i][1] < 100 or arr[i][1] > 440):
            #arr.remove(arr[i])
            #arrx.append(arr[i][0])
            #arry.append(arr[i][1])
            #arrz.append(arr[i][2])
        
        #i += 1

    #i = 0
    ##while(i < len(filter_arr)):
        #filter_arr[i][0] = 0
        #i += 1
    #i = 1
    #c = 0
    
    pc = np.array(arr)

    x_offset =  math.cos(img_no * angle_offset)
    y_offset =  math.sin(img_no * angle_offset)
    #z_offset = 0  # adjust the z-axis offset as desired
    #pc[:, 0] += pc[:, 0] * x_offset + pc[:, 1] * x_offset
    #pc[:, 0].depth_scale = 20
    #pc[:, 0] *= x_offset
    #pc[:, 1] += pc[:, 0] * x_offset - pc[:, 1] * y_offset
    #pc[:, 2] += z_offset
    
    pc[:, 0] = pc[:, 0] * x_offset - pc[:, 1] * y_offset
    pc[:, 1] = pc[:, 1] * x_offset + pc[:, 0] * y_offset

    #rotation_axis = np.array([0, 0, 1])  # rotate around the y-axis
    #rotation_angle = img_no * angle_offset  # adjust the rotation angle as desired
    #rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis*rotation_angle)
    
    #pc[:, :3] = np.matmul(pc[:, :3], rotation_matrix)
    combined_pc = np.vstack((combined_pc, pc))
    ct +=1
    print(ct)

    #points = np.array(filter_arr)
    #np.savetxt(f'shifted_clouds/pc{img_no}.xyz', points, delimiter=' ')

    #input_path= f'shifted_clouds/pc{img_no}.xyz'
    #point_cloud= np.loadtxt(input_path,skiprows=0)

    #pcd = o3d.geometry.PointCloud()
    #pcd.points = o3d.utility.Vector3dVector(pc[:,:3])
    #o3d.visualization.draw_geometries([pcd])
points = combined_pc
cloud = pv.PolyData(points)
cloud.plot(point_size=2)
#np.savetxt(f'combinedpc.xyz')