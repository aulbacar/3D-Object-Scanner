import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

# Load the first image
img0 = cv.imread('Sample_Data/Melatonin_Lit/test0.jpg', cv.IMREAD_GRAYSCALE)

# Initialize SIFT detector
sift = cv.SIFT_create()

# Find the keypoints and descriptors of the first image with SIFT
kp1, des1 = sift.detectAndCompute(img0, None)

# Create a list to store all the keypoints and descriptors
keypoints_list = [kp1]
descriptors_list = [des1]

# Loop through all the other images
for i in range(1, 24):
    # Load the next image
    img = cv.imread(f'Sample_Data/Melatonin_Lit/test{i}.jpg', cv.IMREAD_GRAYSCALE)

    # Find the keypoints and descriptors of the next image with SIFT
    kp2, des2 = sift.detectAndCompute(img, None)

    # Add the keypoints and descriptors to the list
    keypoints_list.append(kp2)
    descriptors_list.append(des2)

    # Create a BFMatcher object
    bf = cv.BFMatcher()

    # Match the descriptors of the most recent image with the descriptors of the current image
    matches = bf.knnMatch(descriptors_list[i-1], des2, k=2)

    # Apply ratio test to get the good matches
    good_matches = []
    for m,n in matches:
        if m.distance < 0.75 * n.distance:
            good_matches.append(m)

    # Get the matched keypoints of the previous image and the current image
    src_pts = np.float32([keypoints_list[i-1][m.queryIdx].pt for m in good_matches]).reshape(-1,1,2)
    dst_pts = np.float32([keypoints_list[i][m.trainIdx].pt for m in good_matches]).reshape(-1,1,2)

    # Find the homography matrix
    M, mask = cv.findHomography(dst_pts, src_pts, cv.RANSAC, 5.0)

    # Apply the homography matrix to the current image to align it with the previous image
    height, width = img0.shape
    aligned_img = cv.warpPerspective(img, M, (width, height))

    # Add the aligned image to the original image
    img0 += aligned_img

# Show the final image
plt.imshow(img0, cmap='gray')
plt.show()
