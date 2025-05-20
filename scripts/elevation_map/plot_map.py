import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import sys
import json
import io
from shapely import wkb
from pathlib import Path


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from scripts.database.rocksdb import RocksDB

def plot_occupancy_map(data, save_path, polygon = None):
    """
    Plots a binary occupancy map and saves it to a file.
    """
    
    plt.figure(figsize=(10, 8))
    plt.imshow(
        data,
        cmap='binary',  # Binary colormap (0 = white, 1 = black)
        vmin=0,
        vmax=1,
        origin='lower',
        aspect='equal'
    )
    
    if polygon:
        x, y = polygon.exterior.xy
        plt.plot(x, y, color='red', linewidth=2)

    # plt.title('Occupancy Map')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    plt.axis('off')
    plt.savefig(save_path, bbox_inches='tight')  # Save the figure to a file
    plt.close()  # Close the figure to free up memory


def plot_elevation_map(data, save_path):
    """
    Plots an elevation map and saves it to a file.
    """
    plt.figure(figsize=(10, 8))
    plt.imshow(
        data,
        cmap='viridis',
        vmax=10,  # Set upper limit
        origin='lower',
        aspect='equal'
    )
    plt.colorbar(label='Elevation')
    plt.title('Elevation Map')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.savefig(save_path, bbox_inches='tight')  # Save the figure to a file
    plt.close()  # Close the figure to free up memory


def plot_map_with_contours(data, polygon, save_path='elevation_with_boundary'):
    """
    Plots a map with an overlay of contour lines.
    
    Parameters:
    - data (2D array-like): The data for which the map and contours are to be plotted.
    """
    plt.figure(figsize=(6, 6))
    plt.imshow(data, cmap='viridis',  origin='lower')
    x, y = polygon.exterior.xy
    plt.plot(x, y, color='red', linewidth=2)
    # plt.title("Map with boundary")
    plt.tight_layout()
    plt.axis('off')
    # plt.colorbar(label='Elevation')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    plt.savefig(save_path, bbox_inches='tight')
    plt.close()


def search_and_read_npy_file(directory, filename=None):
    """
    Searches for a .npy file in the given directory and reads it.
    If a filename is provided, it searches for that specific file.
    Otherwise, it reads the first .npy file found.
    """
    # Search for .npy files in the directory
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith('.npy') and (filename is None or file == filename):
                file_path = os.path.join(root, file)
                print(f"Found .npy file: {file_path}")
                
                # Read the .npy file
                data = np.load(file_path)
                return data

    print("No matching .npy file found.")
    return None


def get_map(date, session, sector, name):
    key = {
        "date": date,
        'session': session,
        'sector': sector,
        'file_name': name
    }   
    raw_data = RocksDB().get(json.dumps(key))
    buffer = io.BytesIO(raw_data)
    return np.load(buffer, allow_pickle=True)

def get_polygon(date, session, sector):
    key = {
        "date": date,
        'session': session,
        'sector': sector,
        'file_name': 'boundary'
    }   
    data = RocksDB().get(json.dumps(key))
    return wkb.loads(data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str)
    parser.add_argument('--date', type=str)
    parser.add_argument('--session',type=str)
    parser.add_argument('--sector', type=str)
    args = parser.parse_args()

    path = Path(args.path, args.date, args.session, args.sector)
    path.mkdir(parents=True, exist_ok=True)

    # elevation_map = search_and_read_npy_file(path, filename='elevation_map.npy')
    # occupancy_map = search_and_read_npy_file(path, filename='occupancy_map.npy')


    raw_elevation_map = get_map(args.date, args.session, args.sector, 'raw_elevation')
    cropped_elevation_map = get_map(args.date, args.session, args.sector, 'cropped_elevation')
    elevation_map = get_map(args.date, args.session, args.sector, 'elevation')
    occupancy_map = get_map(args.date, args.session, args.sector, 'occupancy')
    polygon = get_polygon(args.date, args.session, args.sector)

    plot_elevation_map(raw_elevation_map, os.path.join(path, "raw_elevation_map.png")) # Result should saved in the Same folder
    
    # plot_elevation_map(cropped_elevation_map, os.path.join(path, "cropped_elevation_map.png")) # Result should saved in the Same folder
    plot_elevation_map(elevation_map, os.path.join(path, "elevation_map.png")) # Result should saved in the Same folder
    plot_occupancy_map(occupancy_map, os.path.join(path, "occupancy_map.png"), polygon) # Result should saved in the Same folder

    # plot_map_with_contours(elevation_map, polygon, os.path.join(path, "map_with_boundary.png"))
    # plot_map_with_contours(occupancy_map, polygon, os.path.join(path, "occ_with_boundary.png"))
    plot_map_with_contours(raw_elevation_map, polygon, os.path.join(path, "raw_elevation_map_with_boundary.png")) # Result should saved in the Same folder
    