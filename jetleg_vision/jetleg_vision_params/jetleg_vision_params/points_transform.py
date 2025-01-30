import numpy as np


def load_pointcloud(msg):
    # Parse the bytes as float types
    # Reshape the data to form multiple points
    # This may assume all attributes of the points are the same type
    cloud_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, 8))

    # Number of attributes of the points to be processed
    num_fields = 4

    # Reshape the cloud data to represent a list of points with some
    # number of fields
    cloud_array = cloud_array[:, :, :num_fields]
    cloud_array = cloud_array.reshape((
        cloud_array.shape[0] * cloud_array.shape[1], num_fields
    ))

    cloud_array = cloud_array[np.isfinite(cloud_array).any(axis=1)]
    cloud_array = cloud_array[~np.isnan(cloud_array).any(axis=1)]

    return cloud_array

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