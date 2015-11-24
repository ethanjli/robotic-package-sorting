"""Simulation of the world with virtual world objects."""
import numpy as np

from components.messaging import Broadcaster
from components.concurrency import Reactor
from components.geometry import Pose, Frame, MobileFrame, vector_to_tuple, scale_bounds

class VirtualWorld(Reactor, Broadcaster, Frame):
    """Models a virtual world."""
    def __init__(self, name, world_bounds, canvas, scale=20):
        super(VirtualWorld, self).__init__(name)
        self.__bounds = world_bounds
        self._canvas = canvas
        self.__scale = scale
        self._robots = {}
        self._walls = {}
        self._primitives = {
            "wall": {},
            "wall label": {}
        }

    # Utility for subclasses
    def draw_grid(self, grid_spacing=1):
        """Draws a grid of the specified spacing on the canvas."""
        canvas_bounds = (self.transform_x(self.__bounds[0]),
                         self.transform_y(self.__bounds[3]),
                         self.transform_x(self.__bounds[2]),
                         self.transform_y(self.__bounds[1]))
        # Vertical grid lines
        for x in range(self.__bounds[0], self.__bounds[2] + grid_spacing, grid_spacing):
            transformed = self.transform_x(x)
            self._canvas.create_line(transformed, canvas_bounds[1],
                                     transformed, canvas_bounds[3],
                                     fill="gray", tags=("grid y", "grid"))
        # Horizontal grid lines
        for y in range(self.__bounds[1], self.__bounds[3] + grid_spacing, grid_spacing):
            transformed = self.transform_y(y)
            self._canvas.create_line(canvas_bounds[0], transformed,
                                     canvas_bounds[2], transformed,
                                     fill="gray", tags=("grid x", "grid"))
        # Origin label
        origin_radius = 0.2
        origin_bounds = (self.transform_x(-origin_radius), self.transform_y(-origin_radius),
                         self.transform_x(origin_radius), self.transform_y(origin_radius))
        self._canvas.create_oval(origin_bounds, fill="gray")
    def add_robot(self, virtual_robot):
        self._robots[virtual_robot.get_name()] = virtual_robot
    def add_wall(self, wall):
        """Adds a wall."""
        self._walls[wall.get_id()] = wall
        self._draw_wall(wall)
    def _draw_wall(self, wall):
        """Draws a wall on the canvas."""
        wall_id = wall.get_id()
        wall_rect = self._canvas.create_rectangle(self.transform_bounds(wall.get_bounds()),
                                                  fill="white", tags=("wall"))
        self._primitives["wall"][wall_id] = wall_rect
        wall_label = self._canvas.create_text(vector_to_tuple(self.transform(wall.get_center())),
                                              text=str(wall_id), tags=("wall label"))
        self._primitives["wall label"][wall_id] = wall_label
    def reset(self):
        """Moves everything in the world to its initial position."""
        for (_, robot) in self._robots.items():
            robot.reset_pose()

    # Implementation of parent abstract methods
    def _react(self, signal):
        if signal.Namespace in self._robots:
            if signal.Name == "Pose":
                pass
    def get_pose(self):
        return Pose(np.array([[0], [0]]), 0)
    def _get_scaling(self):
        return (self.__scale, -self.__scale)

class Border(object):
    """Models black border on the floor."""
    pass

class Wall(object):
    """Models an infinitely tall, immobile, rectilinear wall in the virtual world's frame."""
    def __init__(self, wall_id, center_x=0, center_y=0, x_length=10, y_length=4):
        self.__id = wall_id
        self.__bounds = (center_x - 0.5 * x_length, center_y - 0.5 * y_length,
                         center_x + 0.5 * x_length, center_y + 0.5 * y_length)
        center = np.array([[center_x], [center_y]])
        self._center = center
        delta_x = np.array([[x_length], [0]])
        delta_y = np.array([[0], [y_length]])
        self._walls = {
            "East": (center + delta_x - delta_y, center + delta_x + delta_y),
            "North": (center + delta_x + delta_y, center - delta_x + delta_y),
            "West": (center - delta_x + delta_y, center - delta_x - delta_y),
            "South": (center - delta_x - delta_y, center + delta_x - delta_y)
        }

    def get_id(self):
        """Returns the unique wall id of the Wall."""
        return self.__id

    def get_center(self):
        """Returns the center of the Wall."""
        return self._center
    def get_bounds(self):
        """Returns the rectangular bounding box of the Wall."""
        return self.__bounds
    def in_box(self, coord):
        """Checks whether the coordinate is in the box."""
        return (coord[0] >= self.__bounds[0] and coord[0] <= self.__bounds[2]
                and coord[1] >= self.__bounds[1] and coord[2] <= self.__bounds[3])
    def nearest_wall(self, coord):
        """Finds the nearest wall to the coordinate."""
        pass

class Package(MobileFrame):
    """Models a movable box."""
    pass
