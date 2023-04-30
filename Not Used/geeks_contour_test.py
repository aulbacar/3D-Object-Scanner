import cv2
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

# Let's load a simple image with 3 black squares

image = cv2.imread('Sample_Data/Laser_Bottle/test1.jpg')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
#lower_red = np.array([175,50,20])
#upper_red = np.array([180,255,255])
#mask = cv2.inRange(hsv, lower_red, upper_red)
#res = cv2.bitwise_and(image,image, mask= mask)
#cv2.imshow('frame',image)
#cv2.imshow('mask',mask)
#cv2.imshow('res',res)

mask2 = cv2.inRange(hsv, 180, 400)
cv2.imshow(mask2)
cv2.threshold

i = 0
while(i < 1):
     image = cv2.imread('Sample_Data/Laser_Bottle/test' + str(i) + '.jpg')
     i += 1




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

print(contours)
plt.imshow(edged)
plt.show()  
cv2.imshow('Canny Edges After Contouring', edged)
cv2.waitKey(0)

  
print("Number of Contours found = " + str(len(contours)))
  
x_coords = []
for contour in contours:
    for point in contour:
        x_coords.append(point[0][0])
        
# Find the unique values and their counts
unique_values, value_counts = np.unique(x_coords, return_counts=True)

# Sort the counts in descending order and get the indices
sorted_counts_indices = np.argsort(-value_counts)

# Get the two most common values from the sorted indices
most_common_x1 = unique_values[sorted_counts_indices[0]]
most_common_x2 = unique_values[sorted_counts_indices[1]]
  
# Draw all contours
# -1 signifies drawing all contours
cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
#print(contours)
cv2.imshow('Contours', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(most_common_x1)
print(most_common_x2)