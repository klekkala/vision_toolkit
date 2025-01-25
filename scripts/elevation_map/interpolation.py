import numpy as np
import scipy
import scipy.interpolate
import scipy.ndimage
from scipy.ndimage import generic_filter

# https://gis.stackexchange.com/questions/408665/using-python-for-interpolate-data-points-with-scipy-open-for-other-solutions-as


def interpolation(data):
    # return nearest_linear_interpolation(data)
    return rbf_interpolation(data)

def rbf_interpolation(data):
    from scipy.interpolate import Rbf
    x, y = np.indices(data.shape)
    x_flat = x.flatten()
    y_flat = y.flatten()
    data_flat = data.flatten()
    mask = ~np.isnan(data_flat)
    
    rbf = Rbf(x_flat[mask], y_flat[mask], data_flat[mask], function='linear')
    data_interpolated = rbf(x_flat, y_flat).reshape(data.shape)
    return data_interpolated

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


