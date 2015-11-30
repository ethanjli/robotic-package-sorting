"""Support for 2-D geometric operations."""
from collections import namedtuple
from itertools import chain

import numpy as np

# Coord should be a numpy array representing a column vector.
# Angle should be in radians from the frame's +x axis.
Pose = namedtuple("Pose", ["Coord", "Angle"])

# Angles
def normalize_angle(angle):
    """Converts an angle in radians to an angle with -pi < value <= pi.
    This encompasses the output range of the arctan function."""
    negative = angle % -(2 * np.pi)
    return negative - (2 * np.pi * int(negative / np.pi))
def positive_angle(angle):
    """Converts an angle in radians to an angle with 0 <= value < 2 * pi."""
    return angle % (2 * np.pi)

# Vector representations
def to_vector(*values):
    """Converts the input values into a column vector."""
    return np.array([[value] for value in values])
def vector_to_tuple(vector):
    """Converts a column vector into a tuple."""
    return tuple(row[0] for row in vector)
def vectors_to_flat(vectors):
    """Converts iterable of column vectors to flat tuple of alternating coords."""
    return tuple(chain.from_iterable(vector_to_tuple(vector) for vector in vectors))
def homogeneous_form(vector):
    """Returns the homogeneous form of a 2-D column vector."""
    return np.vstack([vector, [1]])
def point_form(homogeneous_vector):
    """Returns the 2-D column vector of two elements from the homogeneous form."""
    return homogeneous_vector[0:2, 0:1]
def direction_vector(angle):
    """Converts an angle from the +x axis into a unit direction vector."""
    return to_vector(np.cos(angle), np.sin(angle))

# Transformation matrices
def rotation_matrix(angle):
    """Converts an angle from the +x axis into a 2-D rotation matrix."""
    return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
def transformation(pose, x_scale=1, y_scale=1):
    """Returns the homogeneous transformation matrix of a frame to its reference."""
    scale_mat = np.array([[x_scale, 0], [0, y_scale]])
    rot_mat = rotation_matrix(pose.Angle)
    transl = pose.Coord
    return np.vstack([np.hstack([np.dot(scale_mat, rot_mat), transl]), [0, 0, 1]])
def compose(transformation_one, transformation_two):
    """Returns the transformation that is the composition of the two inputs."""
    return np.dot(transformation_one, transformation_two)

# Transformations
def transform(matrix, frame_coords):
    """Transforms the non-homogeneous 2-D column vector using the homogeneous transformation matrix."""
    return point_form(np.dot(matrix, homogeneous_form(frame_coords)))
def transform_x(matrix, frame_x):
    """Converts x-coord in the frame to x-coord in the parent's frame."""
    return transform(matrix, to_vector(frame_x, 0))[0][0]
def transform_y(matrix, frame_y):
    """Converts y-coord in the frame to y-coord in the parent's frame."""
    return transform(matrix, to_vector(0, frame_y))[1][0]
def transform_all(matrix, vectors):
    return tuple(transform(matrix, vector) for vector in vectors)

class Frame(object):
    """Mix-in to support coordinate transformations from a frame."""
    def __init__(self):
        super(Frame, self).__init__()

    def get_transformation(self):
        """Returns the transformation matrix for efficient composition of transformations."""
        (x_scale, y_scale) = self._get_scaling()
        return transformation(self.get_pose(), x_scale, y_scale)

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
