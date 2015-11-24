"""Simulation of the world with virtual world objects."""
import numpy as np

from components.messaging import Broadcaster
from components.concurrency import Reactor
from components.geometry import Pose, Frame, MobileFrame
from components.geometry import to_vector, vector_to_tuple, vectors_to_flat
from components.geometry import transform, transform_x, transform_y, transform_all

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
            "wallLabel": {},
            "robotChassis": {},
            "robotChassisLabel": {}
        }

    # Utility for subclasses
    def draw_grid(self, grid_spacing=1):
        """Draws a grid of the specified spacing on the canvas.

        Arguments:
            grid_spacing: the number of cm between each grid line.
        """
        matrix = self.get_transformation_matrix()
        canvas_bounds = (transform_x(matrix, self.__bounds[0]),
                         transform_y(matrix, self.__bounds[3]),
                         transform_x(matrix, self.__bounds[2]),
                         transform_y(matrix, self.__bounds[1]))
        self.__draw_vertical_grid(canvas_bounds, grid_spacing)
        self.__draw_horizontal_grid(canvas_bounds, grid_spacing)
        self.__draw_origin()
    def __draw_vertical_grid(self, canvas_bounds, grid_spacing=1):
        matrix = self.get_transformation_matrix()
        for x in range(self.__bounds[0], self.__bounds[2] + grid_spacing, grid_spacing):
            transformed = transform_x(matrix, x)
            self._canvas.create_line(transformed, canvas_bounds[1],
                                     transformed, canvas_bounds[3],
                                     fill="gray", tags=("gridY", "grid"))
    def __draw_horizontal_grid(self, canvas_bounds, grid_spacing=1):
        matrix = self.get_transformation_matrix()
        for y in range(self.__bounds[1], self.__bounds[3] + grid_spacing, grid_spacing):
            transformed = transform_y(matrix, y)
            self._canvas.create_line(canvas_bounds[0], transformed,
                                     canvas_bounds[2], transformed,
                                     fill="gray", tags=("gridX", "grid"))
    def __draw_origin(self, radius=0.4):
        matrix = self.get_transformation_matrix()
        origin_bounds = (transform_x(matrix, -radius), transform_y(matrix, -radius),
                         transform_x(matrix, radius), transform_y(matrix, radius))
        self._canvas.create_oval(origin_bounds, outline="gray")
    def add_robot(self, virtual_robot):
        """Adds a robot and sets up the appropriate message-passing connections.

        Arguments:
            virtual_robot: a VirtualRobot.
        """
        virtual_robot.register("Position", self)
        self._robots[virtual_robot.get_name()] = virtual_robot
        self.__draw_robot(virtual_robot)
    def __draw_robot(self, virtual_robot):
        robot_name = virtual_robot.get_name()
        matrix = np.dot(self.get_transformation_matrix(), virtual_robot.get_transformation_matrix())
        transformed = vectors_to_flat(transform_all(matrix, virtual_robot.get_corners()))
        chassis_shape = self._canvas.create_polygon(*transformed, fill="gray", outline="black",
                                                    tags=("robotChassis"))
        self._primitives["robotChassis"][robot_name] = chassis_shape
    def add_wall(self, wall):
        """Adds a wall.

        Arguments:
            wall: a Wall.
        """
        self._walls[wall.get_id()] = wall
        self.__draw_wall(wall)
    def __draw_wall(self, wall):
        wall_id = wall.get_id()
        matrix = self.get_transformation_matrix()
        transformed = vectors_to_flat(transform_all(matrix, wall.get_corners()))
        wall_rect = self._canvas.create_polygon(*transformed, fill="white", outline="black",
                                                tags=("wall"))
        self._primitives["wall"][wall_id] = wall_rect
        wall_label = self._canvas.create_text(vector_to_tuple(transform(matrix, wall.get_center())),
                                              text=str(wall_id), tags=("wallLabel"))
        self._primitives["wallLabel"][wall_id] = wall_label
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
        return Pose(to_vector(0, 0), 0)
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
        center = to_vector(center_x, center_y)
        self._center = center
        delta_x = to_vector(0.5 * x_length, 0)
        delta_y = to_vector(0, 0.5 * y_length)
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
    def get_corners(self):
        """Returns a 4-tuple of the corners as column vectors."""
        return (self._walls["East"][0], self._walls["North"][0],
                self._walls["West"][0], self._walls["South"][0])
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
