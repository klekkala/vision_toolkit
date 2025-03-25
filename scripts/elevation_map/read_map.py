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

def get_dense_region_mask(elevation_map, coverage=0.90):
    """
    Returns bounding box covering the densest `coverage` proportion of the elevation map.
    """
    # Step 1: Get non-zero elevation indices
    valid_mask = ~np.isnan(elevation_map)
    y_idxs, x_idxs = np.where(valid_mask)  # row, col

    coords = np.stack([x_idxs, y_idxs], axis=1)  # shape: (N, 2)

    if coords.shape[0] == 0:
        return None  # empty map

    # Step 2: Compute percentiles
    lower_percentile = (1 - coverage) / 2 * 100  # e.g., 5%
    upper_percentile = (1 + coverage) / 2 * 100  # e.g., 95%

    x_min = int(np.percentile(x_idxs, lower_percentile))
    x_max = int(np.percentile(x_idxs, upper_percentile))
    y_min = int(np.percentile(y_idxs, lower_percentile))
    y_max = int(np.percentile(y_idxs, upper_percentile))

    return x_min, x_max, y_min, y_max

def pcl2elevation(points, num_bins = 100):
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
    elevation_map = pcl2elevation(
        points=point_cloud_np,
        num_bins=100
    )
    return elevation_map