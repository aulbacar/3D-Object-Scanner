import numpy as np
import open3d as o3d
import math
import pyvista as pv
from scipy.ndimage.interpolation import rotate

# Define the parameters
num_pcs = 23
angle_offset = (.05) # divide by the number of point clouds
#radius = 100.0  # distance from the center axis

# Initialize the combined point cloud array
combined_pc = np.empty((0, 3))

# Loop through each point cloud file and add the points to the combined point cloud
for i in range(2, 124):
    # Load the current point cloud
    pc_path = f'shifted_clouds/pc{i}.xyz'
    pc = np.loadtxt(pc_path)

    # Apply the desired transformations
    x_offset =  math.sin(i * angle_offset) 
    y_offset =  math.cos(i * angle_offset)
    #z_offset = 0  # adjust the z-axis offset as desired
    #pc[:, 0] = pc[:, 0] * x_offset + pc[:, 1] * x_offset
    #pc[:, 0].depth_scale = 20
    #pc[:, 0] *= x_offset
    #pc[:, 1] = pc[:, 0] * x_offset - pc[:, 1] * y_offset
    #pc[:, 2] += z_offset

    # Apply the desired rotation
    rotation_axis = np.array([0, 0, 1])  # rotate around the y-axis
    rotation_angle = i * angle_offset  # adjust the rotation angle as desired
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis*rotation_angle)
    pc[:, :3] = np.matmul(pc[:, :3], rotation_matrix)
    
    # Add the transformed point cloud to the combined point cloud array
    combined_pc = np.vstack((combined_pc, pc))

# Save the combined point cloud to a file
np.savetxt('combined_pc.xyz', combined_pc, delimiter=' ')
input_path= "combined_pc.xyz"
point_cloud= np.loadtxt(input_path,skiprows=0)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
o3d.visualization.draw_geometries([pcd])
points = np.genfromtxt('combined_pc.xyz', delimiter=" ", dtype=np.float32)
#point_cloud = pv.PolyData(points)
#point_cloud.plot(point_size=15)
cloud = pv.PolyData(points)
cloud.plot(point_size=15)
#cloud.plot(render_points_as_spheres=True)

surf = cloud.delaunay_2d()
surf.plot(show_edges=True)
surf.save('mesh.stl')
#mesh = point_cloud.add_mesh(points)
#mesh.plot()
#mesh.save('mesh.stl')
