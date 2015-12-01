"""Support for 2-D geometric operations.
Mathematics for poses, frames, and coordinate transformations derived from Peter Corke's
"Robotics, Vision, and Control: Fundamental Algorithms in MATLAB".
"""
from collections import namedtuple
from itertools import chain

import numpy as np

from components.util import between, within

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
    rot_scale_mat = np.dot(scale_mat, rot_mat)
    transl = pose.Coord
    return np.vstack([np.hstack([rot_scale_mat, transl]), [0, 0, 1]])
def transformation_inverse(pose, x_scale=1, y_scale=1):
    """Returns the homogeneous transformation matrix into a frame from its reference."""
    scale_mat = np.array([[x_scale, 0], [0, y_scale]])
    rot_mat = rotation_matrix(pose.Angle)
    rot_scale_mat = np.dot(scale_mat, rot_mat).transpose()
    transl = pose.Coord
    return np.vstack([np.hstack([rot_scale_mat, -1 * np.dot(rot_scale_mat, transl)]), [0, 0, 1]])
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

# Geometric primitives
def find_line_intersection(first_point, first_direction, second_point, second_direction):
    """Finds the intersection (if any) between two lines defined by their points and directions.
    Uses the algorithm outlined in Gareth Rees's answer at
    http://stackoverflow.com/questions/563198
    """
    cross_direction = np.cross(first_direction, second_direction, axis=0)
    difference_point = second_point - first_point
    if cross_direction == 0:
        return None # Lines are collinear or parallel
    second_location = float(np.cross(difference_point, first_direction, axis=0)) / cross_direction
    first_location = float(np.cross(difference_point, second_direction, axis=0)) / cross_direction
    return (first_location[0], second_location[0])
def find_ray_segment_intersection(ray_point, ray_angle, segment_left, segment_right):
    """Finds the intersection (if any) between the ray and the segment defined by two endpoints.
    Uses the algorithm outlined in Gareth Rees's answer at
    http://stackoverflow.com/questions/14307158
    """
    intersection = find_line_intersection(ray_point, direction_vector(ray_angle),
                                          segment_left, segment_right - segment_left)
    if intersection is None or intersection[0] < 0 or not between(0, 1, intersection[1]):
        return None
    else:
        return intersection[0]

class Frame(object):
    """Mix-in to support coordinate transformations from a frame."""
    def __init__(self):
        super(Frame, self).__init__()

    def get_transformation(self):
        """Returns the transformation matrix for efficient composition of transformations."""
        (x_scale, y_scale) = self._get_scaling()
        return transformation(self.get_pose(), x_scale, y_scale)
    def get_transformation_inverse(self):
        """Returns the inverse transformation matrix."""
        (x_scale, y_scale) = self._get_scaling()
        return transformation_inverse(self.get_pose(), x_scale, y_scale)

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

class Rectangle(Frame):
    """Models an immobile, rectangular shape."""
    def __init__(self, center_x, center_y, x_length, y_length):
        self.__bounds = (-0.5 * x_length, -0.5 * y_length, 0.5 * x_length, 0.5 * y_length)
        center = to_vector(center_x, center_y)
        self._center = center
        delta_x = to_vector(0.5 * x_length, 0)
        delta_y = to_vector(0, 0.5 * y_length)
        self._sides = {
            "East": (delta_x - delta_y, delta_x + delta_y),
            "North": (delta_x + delta_y, -delta_x + delta_y),
            "West": (-delta_x + delta_y, -delta_x - delta_y),
            "South": (-delta_x - delta_y, delta_x - delta_y)
        }

    # Implementation of parent abstract methods
    def get_pose(self):
        return Pose(self.get_center(), 0)

    def get_center(self):
        """Returns the center of the Wall."""
        return self._center
    def get_corners(self):
        """Returns a 4-tuple of the corners as column vectors."""
        return (self._sides["East"][0], self._sides["North"][0],
                self._sides["West"][0], self._sides["South"][0])

    def in_rectangle(self, coords):
        """Checks whether the coordinate, given as a column vector, is in the Rectangle."""
        transformed = transform(self.get_transformation_inverse(), coords)
        point = vector_to_tuple(transformed)
        return (within(self.__bounds[0], self.__bounds[2], point[0])
                and within(self.__bounds[1], self.__bounds[3], point[1]))
    def nearest_side(self, coords):
        """Finds the nearest side to the coordinate given in the parent frame as a column vector.
        Returns the side as a 2-tuple of coordinates representing a line segment in the
        Rectangle's Frame.
        To identify the nearest side, uses the algorithm outlined in Raymond Manzoni's answer at
        http://math.stackexchange.com/questions/194550/
        """
        transformed = transform(self.get_transformation_inverse(), coords)
        point = vector_to_tuple(transformed)
        slope = abs(float(self.__bounds[3] - self.__bounds[1])
                    / (self.__bounds[2] - self.__bounds[0]))
        if point[1] >= slope * abs(point[0]):
            return self._sides["North"]
        elif point[1] <= -slope * abs(point[0]):
            return self._sides["South"]
        elif slope * point[0] > abs(point[1]):
            return self._sides["East"]
        elif slope * point[0] < -abs(point[1]):
            return self._sides["West"]
    def get_proximity_distance(self, ray_point, ray_angle):
        """Returns the distance to the Rectangle from the given ray, if the ray intersects.
        The ray should be given in the parent frame as a column vector and an angle."""
        matrix = self.get_transformation()
        distances = tuple(find_ray_segment_intersection(ray_point, ray_angle,
                                                        *transform_all(matrix, side))
                          for side in self._sides.values())
        try:
            return min(distance for distance in distances if distance is not None)
        except ValueError:
            return None
