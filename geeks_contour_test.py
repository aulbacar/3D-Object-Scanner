import cv2
import open3d as o3d
import numpy as np

# Let's load a simple image with 3 black squares
import numpy as np

def get_xyz(contour_img, radius, num_images, distance, laser_angle, pixel_to_distance_ratio):
    # Calculate the angle of each image based on the number of images captured
    image_angles = np.linspace(0, 360, num_images, endpoint=False)

    # Calculate the x and y coordinates of each point in the contour map
    x_coords = contour_img[:,:,0] * pixel_to_distance_ratio
    y_coords = contour_img[:,:,1] * pixel_to_distance_ratio

    # Calculate the z coordinate of each point in the contour map
    z_coords = distance * np.cos(np.deg2rad(laser_angle)) - radius * np.sin(np.deg2rad(laser_angle)) - x_coords * np.sin(np.deg2rad(laser_angle))

    # Combine the x, y, and z coordinates into a point cloud
    point_cloud = np.stack((x_coords, y_coords, z_coords), axis=-1)

    return point_cloud

i = 0
while(i < 25):
    image = cv2.imread('Sample_Data/Laser_Bottle/test' + str(i) + '.jpg')

image = cv2.imread('Sample_Data/Keys_Lit/test23.jpg')
cv2.waitKey(0)
  
# Grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  
# Find Canny edges
edged = cv2.Canny(gray, 30, 200)
cv2.waitKey(0)
  
# Finding Contours
# Use a copy of the image e.g. edged.copy()
# since findContours alters the image
contours, hierarchy = cv2.findContours(edged, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
  
cv2.imshow('Canny Edges After Contouring', edged)
cv2.waitKey(0)
  
print("Number of Contours found = " + str(len(contours)))
  
# Draw all contours
# -1 signifies drawing all contours
cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
#print(contours)
cv2.imshow('Contours', image)
cv2.waitKey(0)
cv2.destroyAllWindows()