import cv2
import open3d as o3d
import numpy as np

num_images = 25

def get_xyz(contours, laser_pos, focal_length):
    """
    Given a set of 2D contour points, calculates the corresponding 3D coordinates
    based on the laser's deviation from the X-axis.
    
    Parameters:
        contours (ndarray): An array of shape (N, 2) containing the (x, y) pixel coordinates
                            of the contour points.
        laser_pos (int): The x-coordinate of the laser position in the image.
        focal_length (float): The focal length of the camera used to capture the image.
        
    Returns:
        An array of shape (N, 3) containing the (x, y, z) coordinates of the contour points.
    """
    # Calculate the distance between each contour point and the laser position
    dists = contours[:, 0] - laser_pos
    
    # Calculate the corresponding depth (Z-coordinate) of each point
    depths = dists * focal_length
    
    # Create an array of shape (N, 3) containing the (x, y, z) coordinates of each point
    points_3d = np.zeros((contours.shape[0], 3))
    points_3d[:, 0] = contours[:, 0]  # x-coordinates (same as contour points)
    points_3d[:, 1] = contours[:, 1]  # y-coordinates (same as contour points)
    points_3d[:, 2] = depths          # z-coordinates based on laser deviation from X-axis
    
    return point_cloud

#main
point_clouds = []
for i in range(num_images):
    image = cv2.imread(f'Sample_Data/Laser_Bottle/test{i}.jpg')
    cv2.waitKey(1)

    # Grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find Canny edges
    edged = cv2.Canny(gray, 30, 200)

    # Finding Contours
    # Use a copy of the image e.g. edged.copy()
    # since findContours alters the image
    contours, hierarchy = cv2.findContours(edged,
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # Calculate 3D coordinates for each contour point
    laser_pos = 314  # x-coordinate of the laser position in the image
    focal_length = 3.04  # focal length of the camera used to capture the image
    point_cloud = get_xyz(contours[i], laser_pos, focal_length)

    # Add the resulting point cloud to the list
    point_clouds.append(point_cloud)