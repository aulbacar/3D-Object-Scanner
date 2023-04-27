import numpy as np
import cv2 as cv
import math
import glob
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Load the first image
img1 = cv.imread('Sample_Data/Melatonin_Lit/test0.jpg', cv.IMREAD_GRAYSCALE)

# Initialize SIFT detector
sift = cv.SIFT_create()

# create an empty list to store all the contour coordinates from each image
contours_list = []

# loop through each image file in the specified directory
for filename in glob.glob('Sample_Data/Melatonin_Lit/test0.jpg'):
    # read the image file
    img = cv.imread(filename)
    # convert the image to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # apply a threshold to the grayscale image
    ret, thresh = cv.threshold(gray, 127, 255, 0)
    # find the contours in the thresholded image
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # append the contour coordinates to the list of contours
    contours_list.append(contours[0])

# initialize an empty numpy array to store the x, y, and z coordinates of each point in the contour
points = np.empty((0, 3))

# Find the keypoints and descriptors of the first image with SIFT
kp1, des1 = sift.detectAndCompute(img1, None)

# loop through each contour in the list of contours
for contour in contours_list:
    # loop through each point in the contour
    for point in contour:
        # append the x, y, and z coordinates of the point to the array of points
        points = np.append(points, [[point[0][0], point[0][1], 0]], axis=0)

# calculate the number of images
num_images = len(contours_list)

# calculate the angle increment between each image
angle_increment = 360 / num_images

# loop through each image file in the specified directory
for i in range(num_images):
    # read the image file
    img = cv.imread('Sample_Data/Melatonin_Lit/test{}.jpg'.format(i))
    # convert the image to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # create a SIFT object
    sift = cv.SIFT_create()
    # detect the keypoints and compute the descriptors using SIFT
    kp2, des2 = sift.detectAndCompute(gray, None)
    # create a BFMatcher object and match the descriptors
    bf = cv.BFMatcher()
    matches = bf.match(des1, des2)
    # sort the matches by distance
    matches = sorted(matches, key=lambda x:x.distance)
    # calculate the angle of rotation for this image
    angle = i * angle_increment
    # calculate the rotation matrix for this angle
    rot_mat = cv.getRotationMatrix2D((img.shape[1]/2, img.shape[0]/2), angle, 1)
    # apply the rotation to the array of points
    rotated_points = np.dot(points, rot_mat.T)
    # create a new numpy array to store the x, y, and z coordinates of the rotated points
    rotated_points_3d = np.empty((0, 3))
    # loop through each point in the array of rotated points
    for point in rotated_points:
        # append the x, y, and z coordinates of the rotated point to the new array of rotated points
        rotated_points_3d = np.append(rotated_points_3d, [[point[0], point[1], i]], axis=0)
    # loop through each match and calculate the 3D coordinates of the matched points
    for match in matches:
        # get the coordinates of the matched keypoints in the first and second images
        point1 = kp1[match.queryIdx].pt
        point2 = kp2[match.trainIdx].pt
        # get the x, y, and z coordinates of the matched points in the first image
        x1, y1, z1 = rotated_points_3d[match.queryIdx]
        # get the x, y, and z coordinates of the matched points in the second image
        x2, y2, z2 = rotated_points_3d[match.trainIdx]
        # create a line segment between the two points
        line_seg = [(x1, y1, z1), (x2, y2, z2)]
        # plot the line segment on the 3D plot
        ax.plot(*zip(*line_seg), color='red')

# set the limits of the plot to show the entire object
ax.set_xlim3d(-50, 50)
ax.set_ylim3d(-50, 50)
ax.set_zlim3d(0, num_images)

# show the plot
plt.show()
