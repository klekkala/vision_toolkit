import os
import sys
import open3d as o3d
from interpolation import *
from read_map import *


def build_elevation_occupancy_map(pcd_path, output_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    elevation_map = get_elevation_map(pcd)
    elevation_map = interpolation(elevation_map)
    np.save(os.path.join(output_path, 'elevation_map.npy'), elevation_map)
    
    occupancy_threshold = 2
    occupancy_map = (elevation_map >= occupancy_threshold).astype(int)
    np.save(os.path.join(output_path, 'occupancy_map.npy'), occupancy_map)


if __name__ == "__main__":
    pcd_path = sys.argv[1]     # Input sec5 surfaceMap.pcd
    output_path = sys.argv[2]  # Output folder path resulting elevation Map, occupancy Map, and interpolation resul

    build_elevation_occupancy_map(pcd_path, output_path)

# python build_elevation_occupancy.py C:\Users\andyz\OneDrive\Desktop\Codes\Projects\1.usc-projects\ilab\navisim\navisim\assets\gaussian_splat_data\sec4\surfaceMap.pcd ./sec4/