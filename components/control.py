"""Controls robot motion using sensors and simulation data."""
from collections import namedtuple

import numpy as np

from components.messaging import Signal, Broadcaster
from components.concurrency import Reactor
from components.geometry import normalize_angle, direction_vector
from components.geometry import to_vector, vector_to_tuple, Pose

Motion = namedtuple("Motion", ["Name", "Control", "Direction", "Speed", "Data"])

class PrimitiveController(Reactor, Broadcaster):
    """Takes primitive motion control commands and executes them.

    Signals Received:
        Will react to any Signal of correct name whose Namespace matches the name
        of its robot.
        Pose: Data should be a Pose. Will update the Controller's records of the
        robot's Pose.
        Motion: Data should be a Motion command.

    Motion Commands:
        MoveTo: attempt to move in the robot's current direction to the target x-coord, y-coord,
        or x,y-coord. Assumes the robot is already pointed in the correct direction (use the
        RotateTowards command to achieve this). Data should be a 2-tuple of the target x and y
        coordinates.
        MoveBy: move by the specified distance in the current direction. Data should be a
        2-tuple of x and y offsets.
        RotateTo: rotate to the specified absolute angle ccw from the +x axis. Data should
        be a positive or negative angle in radians, or a 2-tuple of x and y offsets that
        implies the target angle.
        RotateTowards: rotate to the angle that points towards the target x,y-coord. Data
        should be a 2-tuple of the target x and y coordinates.
        RotateBy: rotate by the specified relative ccw angle change. Data should be a
        positive or negative angle in radians.
    Motion Command Control modes:
        DeadReckoning: use only the virtual robot's position for motion control.
    Motion Command Speed should be a positive integer between 0 and 100.
    Motion Command Directions:
        MoveTo, MoveBy:
            Forwards: the robot moves forwards to the target.
            Backwards: the robot moves in reverse to the target.
        RotateTowards:
            Forwards: the robot's front end will point towards the target.
            Backwards: the robot's rear end will point towards the target.

    Signals Broadcast:
        Moved: sent when the last command has been executed. Data is a 2-tuple of the
        last command and the current pose.
    """
    def __init__(self, name, robot, xy_epsilon=0.01, angle_epsilon=(np.pi / 360)):
        super(PrimitiveController, self).__init__(name)
        self._robot = robot
        self._robot.get_virtual().register("Pose", self)
        self._robot_pose = robot.get_virtual().get_pose()
        self._target_pose = None
        self._last_command = None
        self._xy_epsilon = xy_epsilon
        self._angle_epsilon = angle_epsilon

    # Implementation of parent abstract methods
    def _react(self, signal):
        if not signal.Namespace == self._robot.get_name():
            return
        # TODO: react to Stop signal
        if signal.Name == "Pose":
            self._robot_pose = signal.Data
            if self._target_pose is not None and self.__reached_pose():
                self.broadcast(Signal("Moved", self.get_name(), self._robot.get_name(),
                                      (self._last_command, self._robot_pose)))
                self._target_pose = None
                self._robot.move(0)
        elif signal.Name == "Motion" and signal.Data.Control == "DeadReckoning":
            command = signal.Data
            if command.Name == "MoveTo":
                self._last_command = command
                self.__move_to(command.Direction, command.Speed, to_vector(command.Data))
            elif command.Name == "MoveBy":
                self._last_command = command
                current_direction = direction_vector(self._robot_pose.Angle)
                if command.Direction == "Backwards":
                    current_direction = current_direction * -1
                target_coords = command.Data * current_direction + self._robot_pose.Coord
                self.__move_to(command.Direction, command.Speed, target_coords)
            elif command.Name == "RotateTo":
                self._last_command = command
                try:
                    target_angle = np.arctan2(command.Data[1], command.Data[0])
                except TypeError:
                    target_angle = command.Data
                self.__rotate_to(command.Speed, normalize_angle(target_angle))
            elif command.Name == "RotateBy":
                self._last_command = command
                target_angle = normalize_angle(command.Data + self._robot_pose.Angle)
                self.__rotate_to(command.Speed, target_angle)
            elif command.Name == "RotateTowards":
                self._last_command = command
                offset = command.Data - self._robot_pose.Coords
                target_angle = np.arctan2(offset[1], offset[0])
                if command.Direction == "Backwards":
                    target_angle = normalize_angle(target_angle + np.pi)
                self.__rotate_to(command.Speed, target_angle)
    def __reached_pose(self):
        if self._target_pose.Angle is not None:
            return abs(self._target_pose.Angle - self._robot_pose.Angle) < self._angle_epsilon
        else:
            target_coords = vector_to_tuple(self._target_pose.Coord)
            current_coords = vector_to_tuple(self._robot_pose.Coord)
            within_x = (target_coords[0] is None
                        or abs(target_coords[0] - current_coords[0]) < self._xy_epsilon)
            within_y = (target_coords[1] is None
                        or abs(target_coords[1] - current_coords[1]) < self._xy_epsilon)
            return within_x and within_y
    def __move_to(self, direction, speed, target):
        if direction == "Forwards":
            self._robot.move(abs(speed))
        elif direction == "Backwards":
            self._robot.move(-abs(speed))
        self._target_pose = Pose(target, None)
    def __rotate_to(self, speed, target):
        if normalize_angle(target - self._robot_pose.Angle) > 0:
            self._robot.rotate(speed)
        else:
            self._robot.rotate(-speed)
        self._target_pose = Pose(to_vector((None, None)), target)
