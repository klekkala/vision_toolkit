import networkx as nx
import open3d as o3d
import pickle
import time

def load_graph(path):
    with open(path, "rb") as f:
        G = nx.read_gpickle(f)
        return G

def load_meta_info(node):
    trajectory = node['points']
    polygon = node['polygon']
    guassian = node['gaussian']
    occupancy = node['occupancy']
    elevation = node['elevation_map']
    return o3d.visualization.draw_geometries([guassian.to_legacy()])

def benchmark(graph, num_test):
    current = 0
    fps_list = []

    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1280, height=720)

    for i in range(num_test):
        current_node = graph.nodes[current]

        start = time.time()
        # load_meta_info(current_node)
        g = current_node['gaussian'].to_legacy()

        vis.clear_geometries()
        vis.add_geometry(g)
        vis.poll_events()
        vis.update_renderer()

        end = time.time()

        frame_time = end - start
        fps = 1 / frame_time
        fps_list.append(fps)
        
        print(f"Frame time: {frame_time:.4f}s, FPS: {fps:.2f}")

        neighbors = list(graph.neighbors(current))

        if not neighbors:
            print('No more neighbors')
            break

        next_node = neighbors[-1]
        current = next_node
    
    print(f"Average FPS: {sum(fps_list) / len(fps_list):.4f} seconds")

if __name__ == '__main__':
    print('Benchmarking')
    path = '/lab/kiran/vision_toolkit/2023_03_29/0/sequence_graph.gpickle'
    sequence_graph = load_graph(path)
    benchmark(sequence_graph, 100)