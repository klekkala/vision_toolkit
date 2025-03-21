import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import binned_statistic_2d

def smooth_out_point_cloud(points, axis_height, min_height, max_height):
    '''
    Smoothes point cloud by removing abnormalities

    :param points: point cloud coordinates
    :param axis_height: axis of the point cloud that represents the height
    :param max_val: maximum height for clipping
    
    :return: smoothened coorindates
    '''
    points = points[(points[:, axis_height] >= min_height) & (points[:, axis_height] <= max_height)]
    return points

def pointcloud2elevation(points, num_bins = 100):
    '''
    Convert 3d point cloud onto 2d elevation map by plotting height on 2d plane
    Inverse Distance Weighting (IDW), with some adaptations to include nearest-neighbor checks using KDTree and post-processing for smoothing and rounding
    
    :param points: point cloud coordinates
    :param grid_width: width of the 2d plane
    :param grid_height: height of the 2d plane
    :param min_height: minimum height of the original point cloud

    :return: elevation map of the point cloud
    '''
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    elevation_map, x_edges, z_edges, _ = binned_statistic_2d(
        x, z, y, statistic='mean', bins=num_bins
    )

    return elevation_map

def get_elevation_map(point_cloud, height_limit=10):
    '''
    Compute 3d point cloud onto 2d elevation map

    :param point_cloud: 3d point cloud
    :param height_limit: height threshold to clip any height above the limit
    :param grid_resolution: resolution used for visualizing elevation map

    :return: elevation map

        x is the row informations.
        z is the column informations.

        Min offset of row, Min offset of col
        min_bound[index_x], min_bound[index_z]
    '''
    index_x = 0
    index_y = 1 # index of the height coordinate
    index_z = 2

    point_cloud_np = np.asarray(point_cloud.points)
    point_cloud_np = smooth_out_point_cloud(point_cloud_np, axis_height=index_y, min_height=-10, max_height=height_limit)

    elevation_map = pointcloud2elevation(
        points=point_cloud_np,
        num_bins=100
    )
    return elevation_map

def test(points):
    print(points.shape)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]  # Use as color

    plt.figure(figsize=(8, 6))
    sc = plt.scatter(x, z, c=y, cmap='viridis', s=1)  # 'viridis' is good for height maps

    print(x.min(), x.max())
    plt.xlim(x.min(), x.max())
    plt.ylim(z.min(), z.max())

    plt.colorbar(sc, label='Height (Z)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Projection of Point Cloud with Z as Height')
    plt.grid(True)
    plt.show()

    plt.savefig('test.png')