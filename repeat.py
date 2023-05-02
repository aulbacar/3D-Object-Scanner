import cv2 as cv
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt
import pyvista as pv
import math
import trimesh


image = cv.imread('new_data/square_bottle/test/cal.jpg') #find contours to set y limits on important points
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(gray, 127, 255, 0)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
contours = np.vstack(contours).squeeze()
#print(contours)
miny = 480

maxx = 0
i = 0
minx = 640
maxx_bounds = 0
contours[contours[: 1].argsort()] 
while(i < len(contours)):
    if(contours[i][1] < minx): #finding first bright point(illuminated by laser)
        minx = contours[i][0]  #then going down the line until there is a dark spot
    #print(contours[i][1])     #to cut out the laser above the object
    #print("test", contours[1][1])
    #print(c)
    #print(maxx)
    #print(maxy)
    #c = 0
    i += 1
print(minx)

i = 0

#print(contours)
while(i < len(contours)):
    if(contours[i][0] > maxx_bounds): #finding first bright point(illuminated by laser)
        maxx_bounds = contours[i][0]  #then going down the line until there is a dark spot
    #print(contours[i][1])     #to cut out the laser above the object
    #print("test", contours[1][1])
    #print(c)
    #print(maxx)
    #print(maxy)
    #c = 0
    i += 1
#print(maxx)

i = 0
while(i < len(contours)):
    if(contours[i][1] < miny): #finding first bright point(illuminated by laser)
        miny = contours[i][1]  #then going down the line until there is a dark spot
    #print(contours[i][1])     #to cut out the laser above the object
    #print("test", contours[1][1])
    #print(c)
    #print(maxx)
    #print(maxy)
    #c = 0
    i += 1
print(miny)
#miny = 110
#print(contours)
i = 0
maxy = 0
while(i < len(contours)):
    if(contours[i][1] > maxy): #finding first bright point(illuminated by laser)
        maxy = contours[i][1]  #then going down the line until there is a dark spot
    #print(contours[i][1])     #to cut out the laser above the object
    #print("test", contours[1][1])
    #print(c)
    #print(maxx)
    #print(maxy)
    #c = 0
    i += 1
print(maxy)

ct = 0
combined_pc = np.empty((0, 3))
angle_offset = .0505
x_offset = 0
y_offset = 0
for img_no in range(0, 124):
    img = cv.imread(f'new_data/square_bottle/test/test{img_no}.jpg', cv.IMREAD_GRAYSCALE)
    assert img is not None, "file could not be read, check with os.path.exists()"

    ret,th1 = cv.threshold(img,210,255,cv.THRESH_BINARY) #filtering image by brightness


    i = 0
    c = 0
    z = 0

    arr = []
    temp = []
    #miny = 480
    maxx = 0
    max = 0

    #i = 0
    #c = 0
    #minflag = False

    #while(i > len(th1[0])):
        #while(c < len(th1)):
           #print('test')
           #if(th1[c][i] == 255):
                #print(minflag)
                #if(i < miny):
                    #miny = i
                    #print(miny)
                    #minflag = True
                    #print(minflag)
                    #print(c)
           #if(minflag == True and th1[i][c] != 255):
                #minflag = False
                #print(minflag)
                #break
            
           #c += 1
        #c = 0
        #i += 1
    #print(miny)
    i = 0
    c = 0
    while(i < len(th1)): #finds x coordinate where bottom reference laser is shining
        while(c < len(th1[0])):
           if(th1[i][c] == 255):
                if(i > max):
                    max = 640 - c#(c + 200) 
                    scale = int(max * .2)
                    maxx = 640 - (c - (scale)) #image is 640 pixels wide, c is the x value (the loop condition is kind of weird)
                    maxy = i                   #the scalar is there because I found that it gives objects better definition
                    #print(c)
                if(c < maxy):
                    maxy = c
           c += 1
        #print(c)
        #print(maxx)
        #print("maxy", maxy)
        
        c = 0
        i += 1
    #print(c)
    print(maxx)
    print("maxy", maxy)
    #print(maxy)

    i = 0
    c = 0
    while(i < len(th1)): #going through all pixels detected by brightness
        while(c < len(th1[0])):
            if(th1[i][c] == 255):
                #if(i > maxy):
                    #maxy = i
                    #maxx = c
            
                #temp = [c, z, i]
                #temp = [maxx, z, i]
                #arr.append(temp)
                #temp = [0, maxx - c, i]
                #arr.append(temp)
                #temp = [0, maxx - c, i]
                
                if((i < maxy and i > miny) and (c < maxx_bounds and c > minx)): #440 is the bottom laser height
                    temp = [0, maxx-c, i] #x is 0 so the object is rotating around the same area
                    arr.append(temp)      #the depth is based on the x pixel value with reference to where
                                          #the bottom laser (reference laser) is
            
            #arr[i][1] = maxx - arr[i][0]
           
            #temp = [0, maxx - c, i]
            #temp = [c, i-300, i]
            #arr.append(temp)
            
            #print(maxx-c)
            #print(temp)
            #print(maxx-c)
            #if((i) < 440 and (i) > 100):
                #print("added")
                #temp = [417 - c, 0, i]
                #arr.append(temp)
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

    #x_offset =  math.cos(img_no * angle_offset)
    #y_offset =  math.sin(img_no * angle_offset)
    #z_offset = 0  # adjust the z-axis offset as desired
    #pc[:, 0] += pc[:, 0] * x_offset + pc[:, 1] * x_offset
    #pc[:, 0].depth_scale = 20
    #pc[:, 0] *= x_offset
    #pc[:, 1] += pc[:, 0] * x_offset - pc[:, 1] * y_offset
    #pc[:, 2] += z_offset
    
    #pc[:, 0] = pc[:, 0] * x_offset - pc[:, 1] * y_offset
    #pc[:, 1] = pc[:, 1] * x_offset + pc[:, 0] * y_offset

    #as each slice is found it is rotated .05 radians * the image number around the z-axis
    rotation_axis = np.array([0, 0, 1])  # rotate around the y-axis
    rotation_angle = img_no * angle_offset  # adjust the rotation angle as desired
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis*rotation_angle)
    
    pc[:, :3] = np.matmul(pc[:, :3], rotation_matrix)
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

#cloud = pv.PolyData(points) #the point cloud from combining the rotated slices
#cloud.plot(point_size=2)
#surf = cloud.delaunay_2d()


pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(combined_pc[:,:3])
o3d.visualization.draw_geometries([pcd])


# estimate radius for rolling ball
pcd.estimate_normals()

# to obtain a consistent normal orientation
pcd.orient_normals_towards_camera_location(pcd.get_center())

# or you might want to flip the normals to make them point outward, not mandatory
pcd.normals = o3d.utility.Vector3dVector( - np.asarray(pcd.normals))

# surface reconstruction using Poisson reconstruction
mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=4)

# paint uniform color to better visualize, not mandatory
mesh.paint_uniform_color(np.array([0.7, 0.7, 0.7]))

o3d.io.write_triangle_mesh('a.ply', mesh)
#surf.plot(show_edges=True)
#surf.save('mesh.stl') #the printable mesh file
#np.savetxt(f'combinedpc.xyz')