import numpy as np
import open3d as o3d
import math

# Define the parameters
num_pcs = 23
angle_offset =2*( math.pi / num_pcs ) # divide by the number of point clouds
radius = 15 # distance from the center axis

# Initialize the combined point cloud array
combined_pc = np.empty((0, 3))
x_offset = 0
y_offset = 0
# Loop through each point cloud file and add the points to the combined point cloud
for i in range(1, num_pcs+1):
    # Load the current point cloud
    pc_path = f'transformed/rotated_point_cloud{i}.xyz'
    pc = np.loadtxt(pc_path)

    # Apply the desired transformations
    # x_offset += 20
    # y_offset += 20
    x_offset += radius * math.cos(i * angle_offset)
    y_offset += radius * math.sin(i * angle_offset)
    # z_offset = (i - 1) * 0.5  # adjust the z-axis offset as desired
    pc[:, 0] += x_offset
    pc[:, 1] += y_offset
    # pc[:, 2] += z_offset

    # Add the transformed point cloud to the combined point cloud array
    combined_pc = np.vstack((combined_pc, pc))

# Save the combined point cloud to a file
x_axis = [(i, 0, 0) for i in range(0,200)]
y_axis = [(0, i, 0) for i in range(0,300)]
z_axis = [(0, 0, i) for i in range(0,600)]
axes = np.array(x_axis + y_axis + z_axis)

np.savetxt('combined_pc.xyz', combined_pc, delimiter=' ')
np.savetxt(f'rotate_tests/axes.xyz', axes, delimiter=' ')

input_path= "combined_pc.xyz"
input_path1= 'rotate_tests/axes.xyz'

point_cloud= np.loadtxt(input_path,skiprows=0)
point_cloud1= np.loadtxt(input_path1,skiprows=0)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])

pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(point_cloud1[:,:3])

o3d.visualization.draw_geometries([pcd, pcd1])
