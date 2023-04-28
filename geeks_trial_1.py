import cv2
import open3d as o3d
import numpy as np
from sklearn.decomposition import PCA
#12.25 cm radius
# Let's load a simple image with 3 black squares
import numpy as np

def get_xyz(contour_img, radius, num_images, distance, laser_angle, pixel_to_distance_ratio):
    # Calculate the angle of each image based on the number of images captured
    image_angles = np.linspace(0, 360, num_images, endpoint=False)

    # Calculate the x and y coordinates of each point in the contour map
    # x_coords = contour_img[:,:,0] * pixel_to_distance_ratio
    if contour_img.ndim == 3:
        x_coords = contour_img[:,:,0] * pixel_to_distance_ratio
        y_coords = contour_img[:,:,1] * pixel_to_distance_ratio
    elif contour_img.ndim == 2:
        x_coords = contour_img[:,0] * pixel_to_distance_ratio
        y_coords = contour_img[:,1] * pixel_to_distance_ratio
    else:
        raise ValueError(f"Invalid number of dimensions for contour_img: {contour_img.ndim}")
    # y_coords = contour_img[:,:,1] * pixel_to_distance_ratio

    # Calculate the z coordinate of each point in the contour map
    z_coords = distance * np.cos(np.deg2rad(laser_angle)) - radius * np.sin(np.deg2rad(laser_angle)) - x_coords * np.sin(np.deg2rad(laser_angle))

    # Combine the x, y, and z coordinates into a point cloud
    point_cloud = np.stack((x_coords, y_coords, z_coords), axis=-1)

    return point_cloud

def rotate_point_clouds(point_clouds, angle_degrees):
    # Convert angle to radians
    angle = np.deg2rad(angle_degrees)

    # Create rotation matrix around z-axis
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])

    # Rotate each point cloud
    rotated_point_clouds = []
    for point_cloud in point_clouds:
        rotated_point_cloud = np.dot(point_cloud, rotation_matrix)
        rotated_point_clouds.append(rotated_point_cloud)

    return rotated_point_clouds

point_clouds = []

for i in range(1,25):
    image = cv2.imread(f'Sample_Data/Laser_Bottle/test{i}.jpg')
    cv2.waitKey(1)

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

    # Draw all contours
    # -1 signifies drawing all contours
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
    
    # cv2.imwrite(f'contour_maps/contour_map_{i}.jpg', image)
    
    # Get point cloud from contour map
    point_cloud = get_xyz(edged, 12.25, 50, 45, 30, 0.1)

    # Add point cloud to list
    point_clouds.append(point_cloud)

    #print(contours)
    # cv2.imshow('Contours', image)
    # cv2.waitKey(0)
# print(point_clouds)
#get angle for rotaion via PCA
pca = PCA()
pca.fit(point_clouds[0])

# get the third principal component
pc3 = pca.components_[2]

# calculate the angle between pc3 and the z-axis
angle = np.arccos(pc3[2])

# Rotate each point cloud and add to list
rotated_point_clouds = rotate_point_clouds(point_clouds, angle)

# Combine all point clouds into one larger point cloud
point_cloud_combined = np.concatenate(rotated_point_clouds, axis=0)
np.savetxt('test.txt', point_cloud_combined)
# input_path = 'Sample_Data/the_researcher_desk.xyz'
# pc = np.loadtxt(input_path,skiprows=1)
# o3d.io.write_point_cloud("point_ploud.xyz", point_cloud_combined)
# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(pc[:, :3])
pcd.points = o3d.utility.Vector3dVector(point_cloud_combined[:, :3])


o3d.visualization.draw_geometries([pcd])

# cv2.destroyAllWindows()