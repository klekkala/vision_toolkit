import os
import sys
import open3d as o3d
import argparse
import io
import json

from pathlib import Path
from interpolation import *
from read_map import *

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from scripts.database.rocksdb import RocksDB

def build_elevation_occupancy_map(pcd_path, output_path):
    pcd = o3d.io.read_point_cloud(pcd_path.as_posix())
    elevation_map = get_elevation_map(pcd)
    elevation_map = interpolation(elevation_map)
    np.save(os.path.join(output_path, 'elevation_map.npy'), elevation_map)
    
    occupancy_threshold = 2
    occupancy_map = (elevation_map >= occupancy_threshold).astype(int)
    np.save(os.path.join(output_path, 'occupancy_map.npy'), occupancy_map)

    return elevation_map, occupancy_map

def save_map(date, session, sector, map, name):
    buffer = io.BytesIO()
    np.save(buffer, map)
    buffer.seek(0)

    key = {
        "date": date,
        'session': session,
        'sector': sector,
        'file_name': name
    }
    RocksDB().save(json.dumps(key), buffer.read())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str)
    parser.add_argument('--date', type=str)
    parser.add_argument('--session',type=str)
    parser.add_argument('--sector', type=str)
    args = parser.parse_args()

    path = Path(args.path, args.date, args.session, args.sector)
    pcd_path = Path(path, 'sector.pcd')
    elevation_map, occupancy_map = build_elevation_occupancy_map(pcd_path, path)
    save_map(date = args.date, session = args.session, sector = args.sector, map = elevation_map, name = 'elevation')
    save_map(date = args.date, session = args.session, sector = args.sector, map = occupancy_map, name = 'occupancy')
