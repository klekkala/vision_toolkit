import os
import argparse
import numpy as np
import networkx as nx

from visualize_sequence_graph import *
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
from pathlib import Path

def get_sequences(path, date, session):
    directory = Path(path, date, session)
    if not directory.exists():
        raise FileNotFoundError(f"Error: Directory '{directory}' does not exist.")

    sequences = []

    for seq in sorted(os.listdir(directory), key = lambda x : int(x)):
        try:
            pcd, odometry = get_pcd_odometry(Path(path, date, str(session), seq))
            sequences.append((pcd, odometry))
        except FileNotFoundError:
            #TODO(jiwon) : handle error

            continue
    return sequences

    # return [get_pcd_odometry(Path(path, date, str(session), seq)) for seq in sorted(os.listdir(directory), key = lambda x : int(x))]

def get_pcd_odometry(path, pcd_name = 'sequence.pcd', odometry_name='odometry.txt'):
    pcd_path = Path(path, pcd_name)
    odometry_path = Path(path, odometry_name)

    if not pcd_path.exists():
        raise FileNotFoundError(f"Error: '{pcd_path}' does not exist.")

    if not odometry_path.exists():
        raise FileNotFoundError(f"Error: '{odometry_path}' does not exist.")

    return pcd_path, odometry_path


def trajectory2np(trajectory_file):
    data = []
    with open(trajectory_file, 'r') as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 6:
                data.append([float(x) for x in parts])
    return np.array(data)


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

    return g, sequence_map


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

def save_sequence_graph(graph, output):
    if not os.path.exists(output):
        os.makedirs(output)

    nx.write_gpickle(graph, os.path.join(output, "sequence_graph.gpickle"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str)
    parser.add_argument('--date', type=str)
    parser.add_argument('--session',type=str)
    parser.add_argument('--out', type=str)
    parser.add_argument('--display', type=bool, default=True)
    args = parser.parse_args()

    DATE = args.date
    SESSION = args.session
    OUT = args.out

    display = args.display

    sequences = get_sequences(args.path, args.date, args.session)
    np_trajectories = [trajectory2np(trajectory) for (pcd, trajectory) in sequences]
    g, seq_map = build_sequence_graph(np_trajectories)

    output_path = Path(OUT, DATE, SESSION)
    output_path.mkdir(parents=True, exist_ok=True)

    save_sequence_graph(g, output = output_path)

    if display:
        plot_3d_scatter(seq_map, out_path=output_path)
        plot_birdeye_view(seq_map, out_path=output_path)
        plot_seq_graph(g, out_path=output_path)