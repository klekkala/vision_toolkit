import os
import argparse
import sys
import numpy as np
import networkx as nx
import json
import io
import json
import open3d as o3d

from shapely.geometry import Polygon
from shapely import wkb

from visualize_sequence_graph import *
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
from pathlib import Path

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from scripts.database.rocksdb import RocksDB

def get_sectors(path, date, session):
    directory = Path(path, date, session)
    if not directory.exists():
        raise FileNotFoundError(f"Error: Directory '{directory}' does not exist.")

    available_sectors = [i for i in os.listdir(directory) if i.isdigit()]
    sectors = []

    for s in sorted(available_sectors, key = lambda x : int(x)):
        try:
            pcd, trajectory = get_pcd_odometry(Path(path, date, str(session), s))
            np_trajectories = trajectory2np(trajectory)
            elevation, occupancy, polygon = get_meta_data(date, session, s)
            gaussian_splat = get_guassian_splat(date, session, s)
            sectors.append((pcd, np_trajectories, elevation, occupancy, polygon, gaussian_splat))
        except FileNotFoundError:
            #TODO(jiwon) : handle error

            continue
    return sectors

def get_meta_data(date, session, sector):
    elevation_map = get_map(date, session, sector, 'elevation')
    occupancy_map = get_map(date, session, sector, 'occupancy')
    polygon = get_polygon(date, session, sector)

    return elevation_map, occupancy_map, polygon

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

def get_guassian_splat(date, session, sector):
    path = Path('/lab/kiran/navisim/haopeng/splat_files/3dgs_train/output/12a18dcf-1/point_cloud/iteration_30000/point_cloud.ply')
    pcd = o3d.t.io.read_point_cloud(str(path))
    if isinstance(pcd, o3d.cuda.pybind.geometry.PointCloud):
        print("Converting CUDA point cloud to CPU...")
        pcd = pcd.to_legacy()

    return pcd

def get_pcd_odometry(path, pcd_name = 'sector.pcd', odometry_name='odometry.txt'):
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

    for idx, meta_data in enumerate(sequence):
        node_id = idx
        sequence_map[node_id] = meta_data

    g = nx.Graph()
    for id, meta_data in sequence_map.items():
        pcd, trajectory, elevation, occupancy, polygon, gaussian = meta_data
        g.add_node(id, points = trajectory, elevation_map = elevation, occupancy = occupancy, polygon = polygon, gaussian = gaussian)

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

    sectors = get_sectors(args.path, args.date, args.session)
    g, seq_map = build_sequence_graph(sectors)

    output_path = Path(OUT, DATE, SESSION)
    output_path.mkdir(parents=True, exist_ok=True)

    save_sequence_graph(g, output = output_path)

    if display:
        # plot_3d_scatter(seq_map, out_path=output_path)
        # plot_birdeye_view(seq_map, out_path=output_path)
        plot_seq_graph(g, out_path=output_path)