import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import os
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_odometry(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(", ")
            if len(parts) == 7:
                data.append([float(x) if idx != 6 else int(x.replace('.', '')) for idx, x in enumerate(parts)])
    return np.array(data)

def compute_local_poses(positions, orientations):
    """
    Computes local poses relative to the first pose.

    :param positions: Array of shape (N, 3), global positions (x, y, z).
    :param orientations: Array of shape (N, 3), global orientations (yaw, pitch, roll).
    :return: Local positions and local orientations.
    """
    # Compute relative positions
    initial_position = positions[0]
    local_positions = positions - initial_position

    # Compute relative orientations
    initial_rotation = R.from_euler('xyz', orientations[0], degrees=False)
    local_orientations = []
    for orientation in orientations:
        current_rotation = R.from_euler('xyz', orientation, degrees=False)
        relative_rotation = initial_rotation.inv() * current_rotation
        local_orientations.append(relative_rotation.as_euler('xyz', degrees=False))
    local_orientations = np.array(local_orientations)

    return local_positions, local_orientations

def interpolate_positions_scipy(positions, timestamps, target_times):
    """
    Interpolates positions using scipy's interp1d.

    :param positions: Array of shape (N, 3), positions (x, y, z).
    :param timestamps: Array of shape (N,), timestamps for the positions.
    :param target_times: Array of shape (M,), target timestamps for interpolation.
    :return: Interpolated positions (M, 3).
    """
    # Interpolate each coordinate independently
    x_interp = interp1d(timestamps, positions[:, 0], kind='linear', fill_value="extrapolate")
    y_interp = interp1d(timestamps, positions[:, 1], kind='linear', fill_value="extrapolate")
    z_interp = interp1d(timestamps, positions[:, 2], kind='linear', fill_value="extrapolate")

    # Compute interpolated values
    x = x_interp(target_times)
    y = y_interp(target_times)
    z = z_interp(target_times)

    return np.stack((x, y, z), axis=-1)

def interpolate_orientations_scipy(orientations, timestamps, target_times):
    """
    Interpolates orientations using scipy's interp1d (Euler angles).

    :param orientations: Array of shape (N, 3), orientations (yaw, pitch, roll).
    :param timestamps: Array of shape (N,), timestamps for the orientations.
    :param target_times: Array of shape (M,), target timestamps for interpolation.
    :return: Interpolated orientations (M, 3).
    """
    yaw_interp = interp1d(timestamps, orientations[:, 0], kind='linear', fill_value="extrapolate")
    pitch_interp = interp1d(timestamps, orientations[:, 1], kind='linear', fill_value="extrapolate")
    roll_interp = interp1d(timestamps, orientations[:, 2], kind='linear', fill_value="extrapolate")

    yaw = yaw_interp(target_times)
    pitch = pitch_interp(target_times)
    roll = roll_interp(target_times)

    return np.stack((yaw, pitch, roll), axis=-1)

def interpolate_local_poses(positions, orientations, timestamps, target_times):
    """
    Interpolates local poses from odometry data.

    :param file_path: Path to the odometry file.
    :param target_times: Array of target timestamps for interpolation.
    :return: Interpolated positions and orientations (as Euler angles).
    """

    # Compute local poses
    local_positions, local_orientations = compute_local_poses(positions, orientations)

    # Interpolate positions and orientations
    interpolated_positions = interpolate_positions_scipy(local_positions, timestamps, target_times)
    interpolated_orientations = interpolate_orientations_scipy(local_orientations, timestamps, target_times)

    return local_positions, local_orientations, interpolated_positions, interpolated_orientations

def save_interpolated_pose(save_path, timestamps, positions, orientations):
    with open(os.path.join(save_path, 'inter_local_poses.txt'), "w") as f:
        for i in range(len(timestamps)):
            position = positions[i]
            orientation = orientations[i]
            f.write(f"{position[0]}, {position[1]}, {position[2]}, {orientation[0]}, {orientation[1]}, {orientation[2]}, {int(timestamps[i])}\n")

def plot_3d_scatter(positions, title="3D Path Traveled"):
    """
    Plots a 3D scatter graph of x, y, z coordinates to show the traveled paths.

    :param positions: Array of shape (N, 3), where each row is [x, y, z].
    :param title: Title of the 3D plot.
    """
    if positions.shape[1] != 3:
        raise ValueError(f"Expected positions to have shape (N, 3), but got {positions.shape}")


    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Extract x, y, z from positions
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]

    # Create scatter plot
    scatter = ax.scatter(x, y, z, c=range(len(x)), cmap='viridis', s=10)

    # Add a color bar to represent the path sequence
    colorbar = fig.colorbar(scatter, ax=ax)
    colorbar.set_label('Path Progression')

    # Add labels and title
    ax.set_title(title)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')

    plt.savefig(f'{title.lower()}.png', dpi=300, bbox_inches='tight')
    plt.show()



parser = argparse.ArgumentParser()
parser.add_argument('--odo', type=str)
parser.add_argument('--output', type=str)
parser.add_argument('--img', type = str)
parser.add_argument('--display', type = bool)

args = parser.parse_args()
ODOM_FILE = args.odo
OUT_PATH = args.output
IMG_PATH = args.img
display = args.display

name_dict = {}
file_names = []

for fn in os.listdir(IMG_PATH):
    if ".jpg" in fn:
        if int(fn[5:-4]) in name_dict.keys():
            name_dict[int(fn[5:-4])].append(fn)
        else:
            file_names.append(int(fn[5:-4]))
            name_dict[int(fn[5:-4])] = [fn]


odometry_data = read_odometry(ODOM_FILE)
positions = odometry_data[:, 0:3]
orientations = odometry_data[:, 3:6]
time = odometry_data[:, 6]

decimal_places = len(str(time[-1]))
rounded_file_names = []
time_dict = {}
for i in file_names:
    rounded_file_names.append(int(str(i)[:decimal_places]))
    time_dict[rounded_file_names[-1]] = i

rounded_file_names.sort()
interpolation_time = np.array(rounded_file_names)

local_poses, local_orientations, interpolated_positions, interpolated_orientations = interpolate_local_poses(positions, orientations, time, interpolation_time)
# save_interpolated_pose(OUT_PATH, timestamps=time, positions=interpolated_positions, orientations=interpolated_orientations)

if display:
    plot_3d_scatter(positions=local_poses, title='3d_local_poses')
    plot_3d_scatter(positions=interpolated_positions, title='3d_interpolated_poses')
    plot_3d_scatter(positions=positions, title='3d_raw_poses')

#python compute_interpolated_poses.py --display true --odo /lab/tmpig23b/navisim/data/bag_dump/2023_03_11/0/all_odom/odometry.txt --img /lab/tmpig23b/navisim/data/bag_dump/2023_03_11/0/all_imgs --output /lab/tmpig23b/navisim/data/bag_dump/2023_03_11/0/all_odom/inter_local_poses.txt