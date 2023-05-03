import cv2 as cv
import open3d as o3d
import numpy as np

image = cv.imread('sample_data/masking_tape/cal.jpg') #find contours to set y limits on important points
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(gray, 127, 255, 0)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
contours = np.vstack(contours).squeeze()
miny = 480

maxx = 0
i = 0
minx = 640
maxx_bounds = 0
contours[contours[: 1].argsort()] 
while(i < len(contours)):
    if(contours[i][1] < minx): #finding first bright point(illuminated by laser)
        minx = contours[i][0]  #then going down the line until there is a dark spot

    #c = 0
    i += 1


i = 0

#print(contours)
while(i < len(contours)):
    if(contours[i][0] > maxx_bounds): #finding first bright point(illuminated by laser)
        maxx_bounds = contours[i][0]  #then going down the line until there is a dark spot

    #c = 0
    i += 1


i = 0
while(i < len(contours)):
    if(contours[i][1] < miny): #finding first bright point(illuminated by laser)
        miny = contours[i][1]  #then going down the line until there is a dark spot

    i += 1

i = 0
maxy = 0
while(i < len(contours)):
    if(contours[i][1] > maxy): #finding first bright point(illuminated by laser)
        maxy = contours[i][1]  #then going down the line until there is a dark spot

    i += 1


ct = 0
combined_pc = np.empty((0, 3))
angle_offset = .05
x_offset = 0
y_offset = 0
for img_no in range(0, 127):
    img = cv.imread(f'sample_data/masking_tape/test{img_no}.jpg', cv.IMREAD_GRAYSCALE)
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

        
        c = 0
        i += 1


    i = 0
    c = 0
    while(i < len(th1)): #going through all pixels detected by brightness
        while(c < len(th1[0])):
            if(th1[i][c] == 255):

                
                if((i < maxy and i > miny) and (c < maxx_bounds and c > minx)): #440 is the bottom laser height
                    temp = [0, maxx-c, i] #x is 0 so the object is rotating around the same area
                    arr.append(temp)      #the depth is based on the x pixel value with reference to where
                                          #the bottom laser (reference laser) is
            

            c += 1
        
        c = 0
        i += 1

    i = 0
    avg = 0


    i = 0

    
    pc = np.array(arr)



    #as each slice is found it is rotated .05 radians * the image number around the z-axis
    rotation_axis = np.array([0, 0, 1])  # rotate around the y-axis
    rotation_angle = img_no * angle_offset  # adjust the rotation angle as desired
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis*rotation_angle)
    
    pc[:, :3] = np.matmul(pc[:, :3], rotation_matrix)
    combined_pc = np.vstack((combined_pc, pc))
    ct +=1
    print(ct)

points = combined_pc



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
