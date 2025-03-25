import numpy as np
import scipy
import scipy.interpolate
import scipy.ndimage
from scipy.ndimage import binary_dilation, binary_fill_holes
from scipy.interpolate import Rbf
from scipy.ndimage import generic_filter

# https://gis.stackexchange.com/questions/408665/using-python-for-interpolate-data-points-with-scipy-open-for-other-solutions-as


def get_island_mask(elevation_map):
    # Initial mask of valid points (non-NaN)
    valid_mask = ~np.isnan(elevation_map)
    
    # Fill holes to identify the entire island shape (including internal NaNs)
    island_mask = binary_fill_holes(valid_mask)

    return island_mask

def interpolation(data):
    # return nearest_linear_interpolation(data)
    return rbf_interpolation_within_island(data)

def rbf_interpolation_within_island(data, function='linear', smooth=0):
    # Step 1: Create island mask (NaNs inside the island will be interpolated, sea ignored)
    valid_mask = ~np.isnan(data)
    island_mask = binary_fill_holes(valid_mask)

    # Step 2: Separate valid and NaN points within the island only
    y_valid, x_valid = np.where(valid_mask & island_mask)
    z_valid = data[y_valid, x_valid]

    if len(z_valid) < 3:
        raise ValueError("Not enough valid points within island for RBF interpolation.")

    # Step 3: Fit RBF interpolator
    rbf = Rbf(x_valid, y_valid, z_valid, function=function, smooth=smooth)

    # Step 4: Interpolate only NaNs within the island
    y_nan, x_nan = np.where(np.isnan(data) & island_mask)
    z_interp = rbf(x_nan, y_nan)

    # Step 5: Replace NaNs in a copy of the data
    filled_data = data.copy()
    filled_data[y_nan, x_nan] = z_interp

    return filled_data

# def rbf_interpolation(data, function='linear', smooth=0):
#     from scipy.interpolate import Rbf
#     valid_mask = ~np.isnan(data)
#     y_valid, x_valid = np.where(valid_mask)
#     z_valid = data[valid_mask]

#     if len(z_valid) < 3:
#         raise ValueError("Not enough valid points for RBF interpolation.")

#     # RBF interpolator
#     rbf = Rbf(x_valid, y_valid, z_valid, function=function, smooth=smooth)

#     # Interpolate at NaN positions
#     y_nan, x_nan = np.where(np.isnan(data))
#     z_interp = rbf(x_nan, y_nan)

#     # Copy the original map and fill in NaNs
#     filled_map = data.copy()
#     filled_map[y_nan, x_nan] = z_interp
#     return filled_map

# Legacy Interpolation methods
def nearest_linear_interpolation(data):
    mask = np.isnan(data)
    x, y = np.indices(data.shape)
    known_points = np.array((x[~mask], y[~mask])).T
    known_values = data[~mask]
    nan_points = np.array((x[mask], y[mask])).T

    # Initial interpolation using nearest-neighbor to handle edges
    nearest_interpolated_values = scipy.interpolate.griddata(known_points, known_values, nan_points, method='nearest')
    data[mask] = nearest_interpolated_values

    # Refine interpolation using linear method
    mask = np.isnan(data)  # Update mask after nearest interpolation
    if np.any(mask):
        nan_points = np.array((x[mask], y[mask])).T
        linear_interpolated_values = scipy.interpolate.griddata(known_points, known_values, nan_points, method='linear')
        data[mask] = linear_interpolated_values

    return data

# Legacy Interpolation methods
def gaussian_smoothing(data):
    # Apply a stronger Gaussian smoothing
    smoothed_data = scipy.ndimage.gaussian_filter(data, sigma=3)  # Increase sigma from 1 to 3
    return smoothed_data

# Legacy Interpolation methods
def interpolate_nearest(data):
    for _ in range(3):  # Number of iterations
        mask = np.isnan(data)
        data[mask] = generic_filter(data, np.nanmean, size=3, mode='constant', cval=np.NaN)[mask]

    return data


