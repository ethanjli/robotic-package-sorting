"""Support for 2-D geometric operations."""
from collections import namedtuple

import numpy as np

# Coord should be a numpy array representing a column vector.
# Angle should be in radians from the frame's +x axis.
Pose = namedtuple("Pose", ["Coord", "Angle"])

def scale_bounds(bounds, scaling):
    """Scales a bounding box 4-tuple by the scaling factor."""
    return tuple(scaling * bound for bound in bounds)
def vector_to_tuple(vector):
    """Converts a column vector into a tuple."""
    return tuple(row[0] for row in vector)
def homogeneous_form(vector):
    """Returns the homogeneous form of a 2-D column vector."""
    return np.vstack([vector, [1]])
def point_form(homogeneous_vector):
    """Returns the 2-D column vector of two elements from the homogeneous form."""
    return homogeneous_vector[0:2, 0:1]
def direction_vector(angle):
    """Converts an angle from the +x axis into a unit direction vector."""
    return np.array([[np.cos(angle)], [np.sin(angle)]])
def rotation_matrix(angle):
    """Converts an angle from the +x axis into a 2-D rotation matrix."""
    return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
def transformation_matrix(pose, x_scale=1, y_scale=1):
    """Returns the homogeneous transformation matrix of a frame to its reference."""
    scale_mat = np.array([[x_scale, 0], [0, y_scale]])
    rot_mat = rotation_matrix(pose.Angle)
    transl = pose.Coord
    return np.vstack([np.hstack([np.dot(scale_mat, rot_mat), transl]), [0, 0, 1]])
def transform(pose, frame_coords, x_scale=1, y_scale=1):
    """Converts coords in the frame with the given pose to the frame's reference."""
    return point_form(np.dot(transformation_matrix(pose, x_scale, y_scale),
                             homogeneous_form(frame_coords)))

class Frame(object):
    """Mix-in to support coordinate transformations from a frame."""
    def __init__(self):
        super(Frame, self).__init__()

    def transform(self, frame_coords):
        """Converts coords in the frame to coords in the parent's frame."""
        (x_scale, y_scale) = self._get_scaling()
        return transform(self.get_pose(), frame_coords, x_scale, y_scale)
    def transform_x(self, frame_x):
        """Converts x-coord in the frame to x-coord in the parent's frame."""
        return transform(self.get_pose(), np.array([[frame_x], [0]]),
                         self._get_scaling()[0])[0][0]
    def transform_y(self, frame_y):
        """Converts y-coord in the frame to y-coord in the parent's frame."""
        return transform(self.get_pose(), np.array([[0], [frame_y]]),
                         1, self._get_scaling()[1])[1][0]
    def transform_bounds(self, bounds):
        """Converts a 4-tuple bounding box to the bounding box in the parent's frame."""
        return (self.transform_x(bounds[0]), self.transform_y(bounds[1]),
                self.transform_x(bounds[2]), self.transform_y(bounds[3]))

    # Abstract methods
    def get_pose(self):
        """Returns the pose of the Frame relative to its parent Frame."""
        pass
    def _get_scaling(self):
        """Returns a 2-tuple of the x and y scaling relative to its parent Frame."""
        return (1, 1)
class MobileFrame(Frame):
    """Interface for a mobile Frame."""
    def __init__(self):
        super(MobileFrame, self).__init__()

    def reset_pose(self):
        """Resets the frame to its initial pose."""
