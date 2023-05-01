import numpy as np
import open3d as o3d
import math
import pyvista as pv

# Define the parameters
num_pcs = 123
angle_offset = 6.28 / num_pcs  # divide by the number of point clouds
radius = 15 # distance from the center axis

# Initialize the combined point cloud array
combined_pc = np.empty((0, 3))
x_offset = 0
y_offset = 0
# Loop through each point cloud file and add the points to the combined point cloud
for i in range(2, num_pcs+1):
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

#point_cloud = pv.PolyData(combined_pc)
#point_cloud.plot(render_points_as_spheres=True)

cloud = pv.PolyData(combined_pc)
cloud.plot(point_size=15)
# surf = cloud.delaunay_2d()
# surf.plot(show_edges=True)
