import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import os 
from scipy.spatial import cKDTree

def find_closest_nodes(G1, G2, pos1, pos2):
    """
    Finds the closest pair of nodes between two graphs based on Euclidean distance.
    
    Parameters:
        G1, G2: NetworkX graphs
        pos1, pos2: Dictionary of node positions {node: (x, y)}
    
    Returns:
        (node1, node2): The closest pair of nodes between G1 and G2
    """
    min_dist = float('inf')
    closest_pair = (None, None)

    for n1, p1 in pos1.items():
        for n2, p2 in pos2.items():
            dist = np.linalg.norm(np.array(p1) - np.array(p2))  # Euclidean distance
            if dist < min_dist:
                min_dist = dist
                closest_pair = (n1, n2)

    return closest_pair

def merge_and_connect_closest_points(g1, g2):
    # Create a new graph to hold the merged graphs
    combined_graph = nx.compose(g1, g2)

    print(g1.nodes(data=True))

    # Extract points (x, y, z) from the "points" attribute
    # points_g1 = {node: tuple(data['points'][:3]) for node, data in g1.nodes(data=True)}
    # points_g2 = {node: tuple(data['points'][:3]) for node, data in g2.nodes(data=True)}

    # if not points_g1 or not points_g2:
    #     print("One of the graphs has no points.")
    #     return combined_graph  # Return the combined graph without added edges

    # # Convert dictionaries to arrays
    # g1_nodes, g1_coords = zip(*points_g1.items())
    # g2_nodes, g2_coords = zip(*points_g2.items())

    # print(g1_coords)

    # # Build a KD-Tree for efficient nearest neighbor search
    # tree = cKDTree(g2_coords)

    # # Find the nearest neighbor in g2 for each point in g1
    # distances, nearest_indices = tree.query(g1_coords)

    # # Add edges between closest points
    # for i, g1_node in enumerate(g1_nodes):
    #     g2_node = g2_nodes[nearest_indices[i]]
    #     combined_graph.add_edge(g1_node, g2_node, weight=distances[i])

    # print(f"Connected {len(g1_nodes)} nodes from g1 to closest nodes in g2.")

    # return combined_graph 

def plot_seq_graph(graph, date, block_id, title = 'sequence_graph'):
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
    plt.savefig(f'{date}/{block_id}/{title.lower()}.png', dpi=300, bbox_inches='tight')
    plt.show()

g1 = nx.read_gpickle('2023_03_11/0/sequence_graph.gpickle')
g2 = nx.read_gpickle('2023_03_11/1/sequence_graph.gpickle')
g_combined = merge_and_connect_closest_points(g1, g2)
nx.write_gpickle(g_combined, os.path.join('2023_03_11', "sequence_graph_combined.gpickle"))