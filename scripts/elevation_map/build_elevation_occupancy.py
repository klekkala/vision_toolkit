import os
import sys
import open3d as o3d
import argparse

from pathlib import Path

from interpolation import *
from read_map import *


def build_elevation_occupancy_map(pcd_path, output_path):
    pcd = o3d.io.read_point_cloud(pcd_path.as_posix())
    elevation_map = get_elevation_map(pcd)
    elevation_map = interpolation(elevation_map)
    np.save(os.path.join(output_path, 'elevation_map.npy'), elevation_map)
    
    occupancy_threshold = 2
    occupancy_map = (elevation_map >= occupancy_threshold).astype(int)
    np.save(os.path.join(output_path, 'occupancy_map.npy'), occupancy_map)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str)
    parser.add_argument('--date', type=str)
    parser.add_argument('--session',type=str)
    parser.add_argument('--sequence', type=str)
    args = parser.parse_args()

    path = Path(args.path, args.date, args.session, args.sequence)
    pcd_path = Path(path, 'sequence.pcd')
    build_elevation_occupancy_map(pcd_path, path)
