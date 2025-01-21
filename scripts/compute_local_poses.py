import numpy as np
import os
from scipy.spatial.transform import Rotation as R
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_odometry(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(", ")
            if len(parts) == 7:
                data.append([float(x) for x in parts])
    
    return np.array(data)

def read_transformation_matrix(file_path):
    """
        Reads the transformation file and generate a transformation matrix.

        :param matrix_path(str):Path to the transformation.txt file.
        :return: np.ndarray, Transformation matrix.
    """
    # try:
    #     with open(file_path, 'r') as file:
    #         lines = file.readlines()
    #         cleaned_lines = [line.replace('[', '').replace(']', '') for line in lines]
    #         matrix = [[float(val) for val in line.split()] for line in cleaned_lines]
    #     return np.array(matrix)

    # except Exception as e:
    #     print("An error occurred while reading the transformation matrix:")
    #     print(e)
    #     return None
    return np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [0, 0, 0]        
    ])

def compute_local_pose(pose, transformation_matrix):
    return np.dot(pose, transformation_matrix[:-1]) + transformation_matrix[-1]

def euler_to_rotation_matrix(euler_angles):
    r = R.from_euler('xyz', euler_angles, degrees=False)  # Replace 'xyz' with the rotation order if different
    return r.as_matrix()

def save_local_pose(save_path, timestamps, positions, orientations):
    with open(os.path.join(save_path, 'local_poses.txt'), "w") as f:
        for i in range(len(timestamps)):
            position = positions[i]
            orientation = orientations[i]
            f.write(f"{position[0]}, {position[1]}, {position[2]}, {orientation[0]}, {orientation[1]}, {orientation[2]}, {timestamps[i]}\n")

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
    y = positions[:, 2]
    z = positions[:, 1]

    print(len(x))

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
parser.add_argument('--display', type=bool)

args = parser.parse_args()
ODOM_FILE = args.odo
OUT_PATH = args.output
display = args.display

odometry_data = read_odometry(ODOM_FILE)
transformation_matrix = read_transformation_matrix(ODOM_FILE)

positions = odometry_data[:, 0:3]
orientations = odometry_data[:, 3:6]
timestamps = odometry_data[:, 6]

rotation_matrices = [euler_to_rotation_matrix(orientations[i]) for i in range(len(orientations))]
local_positions = compute_local_pose(pose = positions, transformation_matrix=transformation_matrix)
save_local_pose(OUT_PATH, timestamps=timestamps, positions=local_positions, orientations=orientations)

if display:
    plot_3d_scatter(positions=local_positions)