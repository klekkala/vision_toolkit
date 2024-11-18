import numpy as np

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

def point_cloud_to_height_map(points, grid_lower_bound, grid_width, grid_height, min_height, grid_resolution):
    '''
    Convert 3d point cloud onto 2d elevation map by plotting height on 2d plane
    Inverse Distance Weighting (IDW), with some adaptations to include nearest-neighbor checks using KDTree and post-processing for smoothing and rounding
    
    :param points: point cloud coordinates
    :param grid_width: width of the 2d plane
    :param grid_height: height of the 2d plane
    :param min_height: minimum height of the original point cloud
    :param grid_resolution: resolution used to display point cloud on 2d plane

    :return: elevation map of the point cloud
    '''
    index_x = 0
    index_y = 1 # index of the height coordinate
    index_z = 2 

    # Create 2D top-view grid
    _2d_map = np.full((grid_width, grid_height), np.nan)
    # _2d_map = np.full((grid_width, grid_height), -10)

    # Assign elevation values to the grid
    for point in points:
        # Below two lines will plot the x-y coordinates captured by the sensor to the 2D matrix
        x_idx = int((point[index_x] - grid_lower_bound[index_x]) * grid_resolution)
        y_idx = int((point[index_z] - grid_lower_bound[index_z]) * grid_resolution)

        if 0 <= x_idx < grid_width and 0 <= y_idx < grid_height:
            # _2d_map[x_idx, y_idx] = point[index_y] + np.abs(min_height)
            if ((point[index_y] + np.abs(min_height)) < 7):
                _2d_map[x_idx, y_idx] = point[index_y] + np.abs(min_height)

    return _2d_map

def get_elevation_map(point_cloud, height_limit=10, grid_resolution=10):
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

    min_bound = np.rint(point_cloud.get_min_bound()).astype(int)
    max_bound = np.rint(point_cloud.get_max_bound()).astype(int)

    grid_width  = (np.abs(max_bound[index_x]) + np.abs(min_bound[index_x])) * grid_resolution
    grid_height = (np.abs(max_bound[index_z]) + np.abs(min_bound[index_z])) * grid_resolution

    # Extract the highest points
    max_height = np.max(point_cloud_np[:, index_y])
    min_height = np.min(point_cloud_np[:, index_y])

    elevation_map = point_cloud_to_height_map(
        points=point_cloud_np,
        grid_lower_bound=min_bound,
        grid_width=grid_width,
        grid_height=grid_height,
        min_height=min_height,
        grid_resolution=grid_resolution
    )
    
    # min_elevation = np.min(elevation_map[:, index_y])  # Maybe needed??
    
    return elevation_map