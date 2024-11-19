import numpy as np
import os
from scipy.spatial.transform import Rotation as R
import argparse

def read_odometry(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(", ")
            if len(parts) == 7:
                data.append([float(x) for x in parts])
    
    return np.array(data)

def euler_to_rotation_matrix(euler_angles):
    r = R.from_euler('xyz', euler_angles, degrees=False)  # Replace 'xyz' with the rotation order if different
    return r.as_matrix()

def save_local_pose(save_path, timestamps, positions, orientations):
    with open(os.path.join(save_path, 'odometry.txt'), "w") as f:
        for i in range(len(timestamps)):
            position = positions[i]
            orientation = orientations[i]
            f.write(f"{timestamps[i]}, {positions[0]}, {positions[1]}, {positions[2]}, {orientations[0]}, {orientation[1]}, {orientations[2]}\n")


parser = argparse.ArgumentParser()
parser.add_argument('--odo', type=str)
parser.add_argument('--output', type=str)

args = parser.parse_args()
ODOM_FILE = args.odo
OUT_PATH = args.output

odometry_data = read_odometry(ODOM_FILE)
positions = odometry_data[:, 0:3]
orientations = odometry_data[:, 3:6]
timestamps = odometry_data[:, 6]

rotation_matrices = [euler_to_rotation_matrix(orientations[i]) for i in range(len(orientations))]
initial_position = positions[0]
local_positions = positions - initial_position

initial_rotation = R.from_euler('xyz', orientations[0], degrees=False)

local_orientations = []
for i in range(len(orientations)):
    current_rotation = R.from_euler('xyz', orientations[i], degrees=False)
    relative_rotation = initial_rotation.inv() * current_rotation
    local_orientations.append(relative_rotation.as_euler('xyz', degrees=True))  # Convert back to Euler angles

save_local_pose(OUT_PATH, timestamps=timestamps, positions=local_positions, orientations=local_orientations)