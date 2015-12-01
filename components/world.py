"""Simulation of the world with virtual world objects."""
from components.util import rgb_to_hex, clip
from components.messaging import Signal, Broadcaster
from components.concurrency import Reactor
from components.geometry import Pose, Frame, MobileFrame, Rectangle
from components.geometry import to_vector, vector_to_tuple, vectors_to_flat
from components.geometry import transformation, compose
from components.geometry import transform, transform_x, transform_y, transform_all

_FLOOR_BLACK = 20
_FLOOR_WHITE = 90

class VirtualWorld(Reactor, Broadcaster, Frame):
    """Models a virtual world.

    Signals Sent:
        UpdateCoords: Data is a 2-tuple of the canvas item id and a tuple of its new coords.
        UpdateConfig: Data is a 2-tuple of the canvas item id and a dict of its new config values.

    Signals Received:
        Will react to any Signal of correct name.
        Pose: Data should be the Pose of the robot specified in the Namespace.
        Triggers a redrawing of the robot on the canvas.
        Floor: Data should be a 2-tuple of the left and right floor colors, given as RGB 3-tuples.
        Triggers a redrawing of the floor sensors on the canvas.
    """
    def __init__(self, name, world_bounds, canvas, scale=20):
        super(VirtualWorld, self).__init__(name)
        self.__bounds = world_bounds
        self._canvas = canvas
        self.__scale = scale
        self._robots = {}
        self._walls = {}
        self._borders = {}
        self._primitives = {
            "wall": {},
            "wallLabel": {},
            "border": {},
            "borderLabel": {},
            "robotChassis": {},
            "robotFloorLeft": {},
            "robotFloorRight": {}
        }

    # Utility for subclasses
    # Grid
    def draw_grid(self, grid_spacing=1):
        """Draws a grid of the specified spacing on the canvas.

        Arguments:
            grid_spacing: the number of cm between each grid line.
        """
        matrix = self.get_transformation()
        canvas_bounds = (transform_x(matrix, self.__bounds[0]),
                         transform_y(matrix, self.__bounds[3]),
                         transform_x(matrix, self.__bounds[2]),
                         transform_y(matrix, self.__bounds[1]))
        self.__draw_vertical_grid(canvas_bounds, grid_spacing)
        self.__draw_horizontal_grid(canvas_bounds, grid_spacing)
        self.__draw_origin()
    def __draw_vertical_grid(self, canvas_bounds, grid_spacing=1):
        matrix = self.get_transformation()
        for x in range(self.__bounds[0], self.__bounds[2] + grid_spacing, grid_spacing):
            transformed = transform_x(matrix, x)
            self._canvas.create_line(transformed, canvas_bounds[1],
                                     transformed, canvas_bounds[3],
                                     fill="gray", tags=("gridY", "grid"))
    def __draw_horizontal_grid(self, canvas_bounds, grid_spacing=1):
        matrix = self.get_transformation()
        for y in range(self.__bounds[1], self.__bounds[3] + grid_spacing, grid_spacing):
            transformed = transform_y(matrix, y)
            self._canvas.create_line(canvas_bounds[0], transformed,
                                     canvas_bounds[2], transformed,
                                     fill="gray", tags=("gridX", "grid"))
    def __draw_origin(self, radius=0.4):
        matrix = self.get_transformation()
        origin_bounds = (transform_x(matrix, -radius), transform_y(matrix, -radius),
                         transform_x(matrix, radius), transform_y(matrix, radius))
        self._canvas.create_oval(origin_bounds, outline="gray")
    # Robot
    def add_robot(self, robot):
        """Adds a robot and sets up the appropriate message-passing connections.

        Arguments:
            robot: a Robot.
        """
        virtual_robot = robot.get_virtual()
        virtual_robot.register("Pose", self)
        virtual_robot.register("ResetPose", self)
        self._robots[robot.get_name()] = robot
        self.__draw_robot(virtual_robot)
    def __draw_robot(self, virtual_robot):
        robot_name = virtual_robot.get_name()
        matrix = compose(self.get_transformation(), virtual_robot.get_transformation())
        transformed = vectors_to_flat(transform_all(matrix, virtual_robot.get_corners()))
        chassis_shape = self._canvas.create_polygon(*transformed, fill="gray", outline="black",
                                                    tags=("robotChassis"))
        self._primitives["robotChassis"][robot_name] = chassis_shape
        transformed = vectors_to_flat(transform_all(matrix,
                                                    virtual_robot.get_left_floor_corners()))
        left_floor_shape = self._canvas.create_polygon(*transformed, fill="white", outline="white")
        self._primitives["robotFloorLeft"][robot_name] = left_floor_shape
        transformed = vectors_to_flat(transform_all(matrix,
                                                    virtual_robot.get_right_floor_corners()))
        right_floor_shape = self._canvas.create_polygon(*transformed, fill="white", outline="white")
        self._primitives["robotFloorRight"][robot_name] = right_floor_shape
    def __update_robot(self, robot_name, pose):
        virtual_robot = self._robots[robot_name].get_virtual()
        matrix = compose(self.get_transformation(), transformation(pose))
        transformed = transform_all(matrix, virtual_robot.get_corners())
        self.broadcast(Signal("UpdateCoords", self.get_name(), robot_name,
                              (self._primitives["robotChassis"][robot_name],
                               vectors_to_flat(transformed))))
        transformed = transform_all(matrix, virtual_robot.get_left_floor_corners())
        self.broadcast(Signal("UpdateCoords", self.get_name(), robot_name,
                              (self._primitives["robotFloorLeft"][robot_name],
                               vectors_to_flat(transformed))))
        transformed = transform_all(matrix, virtual_robot.get_right_floor_corners())
        self.broadcast(Signal("UpdateCoords", self.get_name(), robot_name,
                              (self._primitives["robotFloorRight"][robot_name],
                               vectors_to_flat(transformed))))
    def __update_robot_floor(self, robot_name, floor_left, floor_right):
        robot = self._robots[robot_name]
        left_rescaled = robot.to_relative_whiteness(floor_left)
        left_hex = rgb_to_hex(left_rescaled, left_rescaled, left_rescaled)
        self.broadcast(Signal("UpdateConfig", self.get_name(), robot_name,
                              (self._primitives["robotFloorLeft"][robot_name],
                               {"fill": left_hex, "outline": left_hex})))
        right_rescaled = robot.to_relative_whiteness(floor_right)
        right_hex = rgb_to_hex(right_rescaled, right_rescaled, right_rescaled)
        self.broadcast(Signal("UpdateConfig", self.get_name(), robot_name,
                              (self._primitives["robotFloorRight"][robot_name],
                               {"fill": right_hex, "outline": right_hex})))
    # Walls
    def add_wall(self, wall):
        """Adds a wall.

        Arguments:
            wall: a Wall.
        """
        self._walls[wall.get_id()] = wall
        self.__draw_wall(wall)
    def __draw_wall(self, wall):
        wall_id = wall.get_id()
        (wall_rect, wall_label) = self.__draw_rectangle(wall)
        self._canvas.itemconfig(wall_rect, fill="white", outline="black", tags=("wall"))
        self._canvas.itemconfig(wall_label, text=str(wall_id), tags=("wallLabel"))
        self._primitives["wall"][wall_id] = wall_rect
        self._primitives["wallLabel"][wall_id] = wall_label
    # Borders
    def add_border(self, border):
        """Adds a border.

        Arguments:
            border: a Border.
        """
        self._borders[border.get_id()] = border
        self.__draw_border(border)
    # Support
    def __draw_border(self, border):
        border_id = border.get_id()
        (border_rect, border_label) = self.__draw_rectangle(border)
        color = rgb_to_hex(border.get_color(), border.get_color(), border.get_color())
        self._canvas.itemconfig(border_rect, fill=color, outline=color, tags=("border"))
        self._canvas.itemconfig(border_label, text=str(border_id),
                                fill="white", tags=("wallLabel"))
        self._primitives["border"][border_id] = border_rect
        self._primitives["borderLabel"][border_id] = border_label
    def __draw_rectangle(self, rectangle):
        matrix = compose(self.get_transformation(), rectangle.get_transformation())
        transformed = vectors_to_flat(transform_all(matrix, rectangle.get_corners()))
        rect = self._canvas.create_polygon(*transformed)
        label = self._canvas.create_text(vector_to_tuple(transform(matrix, to_vector(0, 0))))
        return (rect, label)
    def reset(self):
        """Moves everything in the world to its initial position."""
        for (_, robot) in self._robots.items():
            robot.get_virtual().reset_pose()
    def get_floor_color(self, coords):
        """Determines the floor color at the specified coords, given as a column vector.
        Color returned as a grayscale value between 0 and 255, inclusive."""
        for border in self._borders.values():
            if border.in_rectangle(coords):
                color = border.get_color()
                return color
        return _FLOOR_WHITE
    def get_proximity_distance(self, coords, angle):
        """Determines the distance to the nearest obstacle along the angle from the coords."""
        # TODO: also search packages, not just walls!
        for wall in self._walls.values():
            pass

    # Implementation of parent abstract methods
    def _react(self, signal):
        if signal.Namespace in self._robots:
            if signal.Name == "Pose" or signal.Name == "ResetPose":
                self.__update_robot(signal.Namespace, signal.Data)
            elif signal.Name == "Floor":
                self.__update_robot_floor(signal.Namespace, signal.Data[0], signal.Data[1])
    def get_pose(self):
        return Pose(to_vector(0, 0), 0)
    def _get_scaling(self):
        return (self.__scale, -self.__scale)

class Border(Rectangle):
    """Models black border on the floor."""
    def __init__(self, border_id, color=0, center_x=0, center_y=0, x_length=1, y_length=4):
        super(Border, self).__init__(center_x, center_y, x_length, y_length)
        self.__id = border_id
        self.__color = color

    def get_id(self):
        """Returns the unique border id of the Border."""
        return self.__id
    def get_color(self):
        """Returns the color of the Border."""
        return self.__color

class Wall(Rectangle):
    """Models an infinitely tall box."""
    def __init__(self, wall_id, center_x=0, center_y=0, x_length=10, y_length=4):
        super(Wall, self).__init__(center_x, center_y, x_length, y_length)
        self.__id = wall_id

    def get_id(self):
        """Returns the unique border id of the Wall."""
        return self.__id

class Package(MobileFrame):
    """Models a movable box."""
    pass
