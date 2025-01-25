import os
import argparse
import re
import numpy as np
import networkx as nx
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R

import matplotlib.cm as cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def get_all_dates(root_path):
    """
    Recursively list all files and folders under a given path.

    Args:
        path (str): The root directory to start listing.

    Returns:
        List of all files and folders.
    """
    date_pattern = re.compile(r"^\d{4}_\d{2}_\d{2}$")  # Regex for yyyy_mm_dd
    date_dirs = []

    for entry in os.listdir(root_path):
        full_path = os.path.join(root_path, entry)
        if os.path.isdir(full_path) and date_pattern.match(entry):
            date_dirs.append(full_path)
    
    return date_dirs

def get_all_sessions(date_path):
    sessions = []
    for entry in os.listdir(date_path):
        full_path = os.path.join(date_path, entry)
        sessions.append(os.path.join(full_path, 'all_odom', 'odometry.txt'))
    return sessions


def get_trajectory_files(root_path):
    all_path_dates = get_all_dates(root_path)
    trajectories = []
    
    for path_date in all_path_dates:
        sessions = get_all_sessions(path_date)
        trajectories.extend(sessions)
    
    return trajectories


def create_sequence(trajectory_file):
    data = []
    with open(trajectory_file, 'r') as f:
        for line in f:
            parts = line.strip().split(", ")
            if len(parts) == 7:
                data.append([float(x) if idx != 6 else int(x.replace('.', '')) for idx, x in enumerate(parts)])
    return np.array(data)

def plot_birdeye_view(trajectory_map, title ='sequence_birdeye_view'):
    plt.figure()
    cmap = cm.get_cmap('tab10', len(trajectory_map))  # Use a colormap with a fixed number of colors
    
    for i, (key, positions) in enumerate(trajectory_map.items()):
        # Extract x, y, z from positions
        x = positions[:, 0]
        y = positions[:, 2]
        # color = cmap(i)
        
        # Create scatter plot
        plt.scatter(x, y, s=1, linewidth=0.01)
        plt.text(x[-1], y[-1], key, fontsize = 4, ha = 'right', va='bottom')

    # Add labels and title
    plt.title(title)
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    plt.savefig(f'{title.lower()}.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_3d_scatter(trajectory_map, title="sequence_3D_trajectories"):
    """
    Plots a 3D scatter graph of x, y, z coordinates to show the traveled paths.

    :param positions: Array of shape (N, 3), where each row is [x, y, z].
    :param title: Title of the 3D plot.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    cmap = cm.get_cmap('tab10', len(trajectory_map))  # Use a colormap with a fixed number of colors
    
    for i, (key, positions) in enumerate(trajectory_map.items()):
        # Extract x, y, z from positions
        x = positions[:, 0]
        y = positions[:, 2]
        z = positions[:, 1]
        # color = cmap(i)
        
        # Create scatter plot
        scatter = ax.scatter(x, y, z, label = key, s=1)

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

def build_sequence_graph(sequence):
    sequence_map = {}

    for idx, seq in enumerate(sequence):
        node_id = idx
        if seq.shape != (0,):
            sequence_map[node_id] = seq
    

    g = nx.Graph()
    for id, trajectory in sequence_map.items():
        g.add_node(id, points = trajectory)

    trajectory_ids = list(g.nodes)
    distance_threshold = 1

    for i in range(len(trajectory_ids)):
        for j in range(i + 1, len(trajectory_ids)):
            traj1_id = trajectory_ids[i]
            traj2_id = trajectory_ids[j]
            if _trajectories_cross(g.nodes[traj1_id]['points'], g.nodes[traj2_id]['points'], distance_threshold):
                g.add_edge(traj1_id, traj2_id)

    return g


def _trajectories_cross(points1, points2, distance_threshold):
    """Check if two trajectories cross using KDTree for spatial search."""
    points1_array = np.array([p[:3] for p in points1])  # Extract spatial coordinates
    points2_array = np.array([p[:3] for p in points2])

    # Build KDTree for one set of points
    tree = KDTree(points2_array)
    for i, point1 in enumerate(points1_array):
        distances, indices = tree.query(point1, k=1)
        if distances < distance_threshold:
            return True
    return False

def create_subsequence(sequence, threshold = 17.33):
    """
    :param threshold : maximum time interval per split in seconds
    """
    # Conversion factor: microseconds to seconds
    conversion_factor = 1e6  # For microseconds
    
    # Create a copy of the timestamps in seconds without modifying the original matrix
    timestamps_in_seconds = sequence[:, -1] / conversion_factor

    # Sort the matrix based on the timestamps
    sorted_indices = np.argsort(timestamps_in_seconds)
    sorted_matrix = sequence[sorted_indices]
    timestamps_sorted = timestamps_in_seconds[sorted_indices]
    
    subsequences = []
    current_split = [sorted_matrix[0]] 
    current_start_time = timestamps_sorted[0]  # Track the start time of the current split

    # Iterate through the rows
    for i in range(1, len(sorted_matrix)):
        # Check the time difference in seconds
        if timestamps_sorted[i] - current_start_time < threshold:
            current_split.append(sorted_matrix[i])  # Add the current row to the split
        else:
            # Save the current split and start a new one
            subsequences.append(np.array(current_split))
            current_split = [sorted_matrix[i]]
            current_start_time = timestamps_sorted[i]  # Update the start time

    # Add the last split
    if current_split:
        subsequences.append(np.array(current_split))

    return subsequences

def plot_seq_graph(graph, title = 'sequence_graph'):
    plt.figure(figsize=(8, 6))  # Adjust the figure size
    
    # Get positions for a spring layout (better for dense graphs)
    pos = nx.spring_layout(graph)  
    
    # Draw the graph
    nx.draw(
        graph,
        pos,
        with_labels=True,
        node_size=100,
        node_color="skyblue",
        font_size=5,
        font_color="black",
        edge_color="gray",
    )
    
    # Add a title
    plt.title(title, fontsize=14)
    plt.savefig(f'{title.lower()}.png', dpi=300, bbox_inches='tight')
    plt.show()

def save_sequence_graph(graph, output):
    nx.write_gpickle(graph, os.path.join(output, "sequence_graph.gpickle"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--odo', type=str)
    parser.add_argument('--output', type=str)
    parser.add_argument('--display_result', type=bool, default=True)
    args = parser.parse_args()

    ODOM_DIR = args.odo
    OUT_PATH = args.output
    display = args.display

    trajectory_files = get_trajectory_files(ODOM_DIR)
    sequences = [create_sequence(trajectory) for trajectory in trajectory_files]

    for seq in sequences:
        sub_sequences = create_subsequence(seq)

        for sub_sequence in sub_sequences:
            g = build_sequence_graph(sub_sequences)


    # longest_sequence = max(sequences, key=len)
    # subsequence = create_subsequence(longest_sequence)


    # sector_odometries = list_sector_odom(ODOM_DIR)
    # sequence_graph = build_sequence_graph(sector_odometries)
    # if display:
    #     plot_graph_2d(sequence_graph)
    #     plot_sequence_graph_3d(sequence_graph)