import argparse
import os
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt


def plot_occupancy_map(data, save_path):
    """
    Plots a binary occupancy map and saves it to a file.
    """
    plt.figure(figsize=(10, 8))
    plt.imshow(
        data,
        cmap='binary',  # Binary colormap (0 = white, 1 = black)
        vmin=0,
        vmax=1,
        origin='lower',
        aspect='equal'
    )
    plt.title('Occupancy Map')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.savefig(save_path, bbox_inches='tight')  # Save the figure to a file
    plt.close()  # Close the figure to free up memory


def plot_elevation_map(data, save_path):
    """
    Plots an elevation map and saves it to a file.
    """
    plt.figure(figsize=(10, 8))
    plt.imshow(
        data,
        cmap='terrain',
        vmax=10,  # Set upper limit
        origin='lower',
        aspect='equal'
    )
    plt.colorbar(label='Elevation')
    plt.title('Elevation Map')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.savefig(save_path, bbox_inches='tight')  # Save the figure to a file
    plt.close()  # Close the figure to free up memory


def plot_map_with_contours(data):
    """
    Plots a map with an overlay of contour lines.
    
    Parameters:
    - data (2D array-like): The data for which the map and contours are to be plotted.
    """
    plt.figure(figsize=(10, 8))

    # Plot the heatmap
    plt.imshow(
        data,
        cmap='terrain',
        origin='lower',
        aspect='auto'
    )
    plt.colorbar(label='Elevation')

    # Generate x and y coordinates
    x = np.arange(data.shape[1])
    y = np.arange(data.shape[0])
    X, Y = np.meshgrid(x, y)

    # Overlay contour lines on the same figure
    CS = plt.contour(
        X, Y, data,
        levels=20,         # Adjust number of contour levels as needed
        colors='k',        # Contour line color
        linewidths=0.5     # Line width for contours
    )

    # plt.clabel(CS, inline=True, fontsize=8, fmt='%.1f')  # Label contours

    plt.title('Smoothed Elevation Map with Contours')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


def search_and_read_npy_file(directory, filename=None):
    """
    Searches for a .npy file in the given directory and reads it.
    If a filename is provided, it searches for that specific file.
    Otherwise, it reads the first .npy file found.
    """
    # Search for .npy files in the directory
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith('.npy') and (filename is None or file == filename):
                file_path = os.path.join(root, file)
                print(f"Found .npy file: {file_path}")
                
                # Read the .npy file
                data = np.load(file_path)
                return data

    print("No matching .npy file found.")
    return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str)
    parser.add_argument('--date', type=str)
    parser.add_argument('--session',type=str)
    parser.add_argument('--sequence', type=str)
    args = parser.parse_args()

    path = Path(args.path, args.date, args.session, args.sequence)

    elevation_map = search_and_read_npy_file(path, filename='elevation_map.npy')
    occupancy_map = search_and_read_npy_file(path, filename='occupancy_map.npy')

    plot_elevation_map(elevation_map, os.path.join(path, "elevation_map.png")) # Result should saved in the Same folder
    plot_occupancy_map(occupancy_map, os.path.join(path, "occupancy_map.png")) # Result should saved in the Same folder

# python plot_map.py ./sec4/