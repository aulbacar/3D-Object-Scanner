import cv2
import numpy as np

# Define camera intrinsics
focal_length = 500
laser_pos = np.array([0, , -10])

# Initialize empty point cloud
point_cloud = np.empty((0, 3))

# Loop over all images
for i in range(1, 26):
    # Load image
    img = cv2.imread(f"Sample_Data/test{i}.jpg")
    
    # Convert to grayscale and apply thresholding
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 127, 255, 0)
    
    # Find contours in the thresholded image
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Loop over all contours in the image
    for contour in contours:
        # Generate point cloud from contour
        xyz = np.empty((len(contour), 3))
        xyz[:, 0] = contour[:, 0][:, 0]  # x-coordinates (same as contour points)
        xyz[:, 1] = contour[:, 0][:, 1]  # y-coordinates (same as contour points)
        xyz[:, 2] = -focal_length  # z-coordinates (distance from camera)
        xyz += laser_pos  # add laser position to get world coordinates
        
        # Add points to point cloud
        point_cloud = np.vstack((point_cloud, xyz))

# Save point cloud to file
np.savetxt("point_cloud.csv", point_cloud, delimiter=",")
