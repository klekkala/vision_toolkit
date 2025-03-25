import os
import sys
import open3d as o3d
import argparse
import io
import json
import copy

from shapely.geometry import Polygon
from shapely import wkb
from skimage import measure
from skimage import filters

from pathlib import Path
from interpolation import *
from read_map import *

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from scripts.database.rocksdb import RocksDB

def _get_boundary_polygon(elevation_map, name):
    """
    Generate a boundary polygon for all coordinates with value 1 in the occupancy map.

    :param occupancy_map: 2D numpy array, Occupancy map with binary values (0 and 1).
    :return: Polygon, Shapely Polygon object representing the boundary.
    """
    valid_mask = ~np.isnan(elevation_map)

    # Step 2: Label connected regions in the valid mask
    labeled_mask = measure.label(valid_mask, connectivity=2)

    # Step 3: Keep only the largest connected component
    props = measure.regionprops(labeled_mask)
    if props:
        largest_region = max(props, key=lambda x: x.area)
        main_region_mask = labeled_mask == largest_region.label
    else:
        main_region_mask = np.zeros_like(valid_mask, dtype=bool)

    padded_mask = np.pad(main_region_mask, pad_width=1, mode='constant', constant_values=False)
    contours = measure.find_contours(padded_mask.astype(float), level=0.5)
    contours = [contour - 1 for contour in contours]
    main_contour = max(contours, key=lambda c: c.shape[0])
    contour_polygon = Polygon([(x, y) for y, x in main_contour])
    
    return contour_polygon


def build_elevation_occupancy_map(pcd_path, output_path):
    pcd = o3d.io.read_point_cloud(pcd_path.as_posix())
    elevation_map = get_elevation_map(pcd)
    
    #TODO(jiwon-hae) : Remove raw elevation map after testing
    raw_elevation_map = elevation_map.copy()
    bbox = get_dense_region_mask(elevation_map=elevation_map, coverage=0.95)
    if bbox is None:
        raise ValueError("No nonna elevation data found in the elevation map.")
    
    x_min, x_max, y_min, y_max = bbox
    elevation_map = elevation_map[y_min:y_max, x_min:x_max]

    #TODO(jiwon-hae) : Remove cropped elevation map after testing
    cropped_elevation = elevation_map.copy()
    
    elevation_map = interpolation(elevation_map)
    boundary_polygon = _get_boundary_polygon(cropped_elevation, name = 'interpolated')

    occupancy_threshold = 2
    occupancy_map = (elevation_map >= occupancy_threshold).astype(int)
    
    return raw_elevation_map, cropped_elevation, elevation_map, occupancy_map, boundary_polygon

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

def save_polygon(date, session,sector, polygon):
    polygon_bytes = wkb.dumps(polygon)
    key = {
        "date": date,
        'session': session,
        'sector': sector,
        'file_name': 'boundary'
    }
    RocksDB().save(json.dumps(key), polygon_bytes)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str)
    parser.add_argument('--date', type=str)
    parser.add_argument('--session',type=str)
    parser.add_argument('--sector', type=str)
    args = parser.parse_args()

    print(f'Building elevation/occupancy map for {args.date}/{args.session}, sector{args.sector}')

    path = Path(args.path, args.date, args.session, args.sector)
    pcd_path = Path(path, 'sector.pcd')
    raw_elevation_map, cropped_elevation, elevation_map, occupancy_map, boundary_polygon = build_elevation_occupancy_map(pcd_path, path)

    save_map(date = args.date, session = args.session, sector = args.sector, map = raw_elevation_map, name = 'raw_elevation')
    save_map(date = args.date, session = args.session, sector = args.sector, map = cropped_elevation, name = 'cropped_elevation')
    save_map(date = args.date, session = args.session, sector = args.sector, map = elevation_map, name = 'elevation')
    save_map(date = args.date, session = args.session, sector = args.sector, map = occupancy_map, name = 'occupancy')
    save_polygon(date = args.date, session = args.session, sector = args.sector, polygon = boundary_polygon)
