
import networkx as nx
import matplotlib.cm as cm
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

def plot_birdeye_view(trajectory_map, out_path, title ='sequence_birdeye_view'):
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

    plt.savefig(f'{out_path}/{title.lower()}.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_3d_scatter(trajectory_map, out_path, title="sequence_3D_trajectories"):
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

    plt.savefig(f'{out_path}/{title.lower()}.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_seq_graph(graph, out_path, title = 'sequence_graph'):
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
    plt.savefig(f'{out_path}/{title.lower()}.png', dpi=300, bbox_inches='tight')
    plt.show()