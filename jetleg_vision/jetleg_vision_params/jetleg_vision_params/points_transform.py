import numpy as np


def restrict_pointcloud(cloud_array, x_domain, y_domain):

    # cloud array is (N x 3) array, with each row being [x, y, z]
    # sort by x,y coordinates into heightmap image pixels

    # clip point cloud according to current position

    ## Lateral (left/right) direction
    x_minimum = x_domain[0]
    x_maximum = x_domain[1]

    ## Forward direction
    y_minimum = y_domain[0]
    y_maximum = y_domain[1]

    # Remove invalid points
    cloud_array = cloud_array[np.isfinite(cloud_array).any(axis=1)]
    cloud_array = cloud_array[~np.isnan(cloud_array).any(axis=1)]

    # y view restriction
    cloud_restricted = cloud_array[np.where(cloud_array[:, 1] <= y_maximum)]
    cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 1] >= y_minimum)]

    # x view restriction
    cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 0] <= x_maximum)]
    cloud_restricted = cloud_restricted[np.where(cloud_restricted[:, 0] >= x_minimum)]

    return cloud_restricted