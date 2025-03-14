import open3d as o3d
import numpy as np
from pathlib import Path

from scipy.spatial.transform import Rotation as R

def get_visible_points(pcd, poses):
    visible_points = set()  # Use a set to avoid duplicate indices
    bbox = pcd.get_axis_aligned_bounding_box()
    radius = bbox.get_max_extent() * 1.5  # Adjust radius dynamically

    for pose in poses:
        camera_position = pose[:3, 3]  # Extract translation vector
        _, pt_map = pcd.hidden_point_removal(camera_position, radius)
        visible_points.update(pt_map)  # Store indices of visible points

    return pcd.select_by_index(list(visible_points))  # Filter point cloud

def save_point_cloud(output_path):
    success = o3d.io.write_point_cloud(output_path, visible_pcd, write_ascii=False, compressed=False)

    if success:
        print(f"Point cloud successfully saved to {output_path}")
    else:
        print("Failed to save the point cloud.")

def read_odometry(file_path):
    poses = []
    timestamps = []
    with open(file_path, 'r') as f:
        for line in f:
            values = list(map(float, line.strip().split(',')))
            x, y, z = values[0:3]
            roll, pitch, yaw = values[3:6]
            timestamp = values[6]  # Assuming timestamp is the first value
            
            rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
            
            pose = np.eye(4)
            pose[:3, :3] = rotation_matrix
            pose[:3, 3] = [x, y, z]
            poses.append(pose)
            timestamps.append(timestamp)
    return poses, timestamps

def filter_poses_by_time_window(poses, timestamps, start_time, window_size):
    end_time = start_time + window_size
    return [pose for pose, t in zip(poses, timestamps) if start_time <= t < end_time]

def sort_poses_by_timestamp(poses, timestamps):
    # Combine poses and timestamps into a list of tuples
    combined = list(zip(timestamps, poses))
    
    # Sort the combined list by timestamp
    combined.sort(key=lambda x: x[0])
    
    # Separate the sorted timestamps and poses
    sorted_timestamps, sorted_poses = zip(*combined)
    
    return list(sorted_poses), list(sorted_timestamps)

date = '2023_03_11'
total_sessions = 1

for session in range(total_sessions):
    pcd_path = f'/lab/tmpig23b/vision_toolkit/data/bag_dump/{date}/{session}'
    pcd = o3d.io.read_point_cloud(f'{pcd_path}/all_lego/surfaceMap.pcd')
    poses, timestamps = read_odometry(f'{pcd_path}/all_odom/odometry.txt')
    poses, timestamps = sort_poses_by_timestamp(poses, timestamps)

    window_size = 20  # 50 seconds
    step_size = 5  # 5 seconds overlap between windows

    start_time = min(timestamps)
    index = 0

    Path(f"{date}/{session}").mkdir(parents=True, exist_ok=True)

    while start_time <= max(timestamps):
        window_poses = filter_poses_by_time_window(poses, timestamps, start_time, window_size)
        visible_pcd = get_visible_points(pcd, window_poses)
        save_point_cloud(f'{date}/{session}/sequence{index}.pcd')
        index += 1
        start_time += window_size
