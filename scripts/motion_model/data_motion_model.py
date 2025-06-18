import sys
import argparse

from pathlib import Path

# Add the scripts/ folder to the Python path
scripts_path = Path(__file__).resolve().parent.parent
sys.path.append(str(scripts_path))

from rosbags.highlevel import AnyReader

from elevation_map.read_map import get_elevation_map

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import open3d as o3d

def get_bags(date):
    return [f for f in Path(f'/data/{date}/cam1').glob('*.bag') if f.is_file()]

def get_odometry(date, session):
    odom_path = Path(f'/lab/tmpig23b/vision_toolkit/data/bag_dump/{date}/{session}/all_odom/odometry.txt')
    if not odom_path.exists():
        raise FileNotFoundError(f"Odometry file not found: {odom_path}")
    
    df = pd.read_csv(odom_path, sep=',', header=None)
    df.columns = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'timestamp']
    return df

def read_cmd_vel(bagpath, topic_name='/cmd_vel'):
    from math import isclose
    bagpath = Path(bagpath)
    data = {
        'timestamp': [],
        'linear_x': [],
        'linear_y': [],
        'linear_z': [],
        'angular_x': [],
        'angular_y': [],
        'angular_z': [],
    }

    with AnyReader([bagpath]) as reader:
        connections = {conn.topic: conn for conn in reader.connections}
        if topic_name not in connections:
            raise ValueError(f"Topic '{topic_name}' not found in the bag file.")

        conn = connections[topic_name]

        for conn, timestamp, rawdata in reader.messages(connections=[conn]):
            msg = reader.deserialize(rawdata, conn.msgtype)
            data['timestamp'].append(timestamp / 1e9)  # ns → sec
            data['linear_x'].append(msg.linear.x)
            data['linear_y'].append(msg.linear.y)
            data['linear_z'].append(msg.linear.z)
            data['angular_x'].append(msg.angular.x)
            data['angular_y'].append(msg.angular.y)
            data['angular_z'].append(msg.angular.z)

    return pd.DataFrame(data)

def get_all_topics(bagpath):
    with AnyReader([bagpath]) as reader:
        return [conn.topic for conn in reader.connections]

def generate_elevation_map(date, session):
    surface_map_path = Path(f'/lab/tmpig23b/vision_toolkit/data/bag_dump/{date}/{session}/all_lego/surfaceMap.pcd')
    print(surface_map_path)
    
    if not surface_map_path.exists():
        raise FileNotFoundError(f"Surface map not found at: {surface_map_path}")

    pcd = o3d.io.read_point_cloud(str(surface_map_path))
    if pcd.is_empty():
        raise ValueError(f"Loaded point cloud is empty: {surface_map_path}")

    print(f"[INFO] Loaded point cloud with {len(pcd.points)} points")

    elevation_map = get_elevation_map(pcd)
    return elevation_map

def visualize(npy):
    plt.imshow(npy, cmap='viridis')  # Or use 'gray', 'hot', etc.
    plt.colorbar(label='Value')
    plt.title("2D Array Visualization")
    plt.show()
    plt.savefig('elevation_map.png')


def main():
    parser = argparse.ArgumentParser(description="Process motion and elevation data from ROS bags.")
    parser.add_argument('--date', type=str, required=True, help='Date folder (e.g., 2023_03_29)')
    parser.add_argument('--session', type=int, required=True, help='Session number (e.g., 0)')
    parser.add_argument('--output_folder', type=str, required=True, help='Output folder name')

    args = parser.parse_args()
    date = args.date
    session = args.session
    
    bags = get_bags(date)
    bagpath = bags[session]

    topics = get_all_topics(bagpath)
    print(topics)

    control_signals = read_cmd_vel(bagpath)
    odometry = get_odometry(date, session)

    # Merge based on nearest timestamp
    combined = pd.merge_asof(control_signals.sort_values('timestamp'),
                            odometry.sort_values('timestamp'),
                            on='timestamp',
                            direction='nearest')

    combined.to_csv("trajectory_info.csv", index=False)

    elevation_map = generate_elevation_map(date, session)
    np.save('elevation_map.npy', elevation_map)
    visualize(elevation_map)

