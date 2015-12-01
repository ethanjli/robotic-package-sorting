"""Controls robot motion using sensors and simulation data."""
from collections import namedtuple

import numpy as np

from components.util import within, initialized_coroutine
from components.messaging import Signal, Broadcaster
from components.concurrency import Reactor
from components.geometry import normalize_angle, positive_angle, direction_vector
from components.geometry import to_vector, vector_to_tuple, Pose

Motion = namedtuple("Motion", ["Name", "Control", "Direction", "Speed", "Data"])

class PrimitiveController(Reactor, Broadcaster):
    """Takes primitive motion control commands and executes them.

    Signals Received:
        Will react to any Signal of correct name whose Namespace matches the name
        of its robot.
        Pose: Data should be a Pose. Will update the Controller's records of the
        robot's Pose and check if the current motion, if it exists, has finished.
        ResetPose: Data should be a Pose. Will update the Controller's records of the
        robot's Pose.
        Motion: Data should be a Motion command.
        Stop: stops the robot and cancels the active Motion command, if applicable.
        Pause: stops the robot and pauses the active Motion command, if applicable.
        Resume: resumes the paused active Motion command, if applicable.

    Motion Commands:
        MoveTo: attempt to move in the robot's current direction to the target x-coord, y-coord,
        or x,y-coord. Assumes the robot is already pointed in the correct direction (use the
        RotateTowards command to achieve this). Data should be a 2-tuple of the target x and y
        coordinates; to target only x-coord, give None as the y-coord; to target only y-coord,
        give None as the x-coord.
        MoveBy: move by the specified distance in the current direction. Data should be the
        distance to move.
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
            1: the robot moves forwards to the target.
            -1: the robot moves in reverse to the target.
        RotateTowards:
            1: the robot's front end will point towards the target.
            -1: the robot's rear end will point towards the target.

    Signals Broadcast:
        Moved: sent when the last command has been executed. Data is a 2-tuple of the
        last command and the current pose.
    """
    def __init__(self, name, robot):
        super(PrimitiveController, self).__init__(name)
        self._robot = robot
        self._robot.get_virtual().register("Pose", self)
        self._robot.get_virtual().register("ResetPose", self)
        self._robot_pose = robot.get_virtual().get_pose()
        self._previous_pose = self._robot_pose
        self._target_pose = None
        self._last_command = None

    # Implementation of parent abstract methods
    def _react(self, signal):
        if not signal.Namespace == self._robot.get_name():
            return
        if signal.Name == "Stop":
            self.__finish_motion(False)
        elif signal.Name == "Pause":
            self._robot.move(0)
        elif (signal.Name == "Resume" and self._target_pose is not None
              and self._last_command is not None):
            command = self._last_command
            if command.Name == "MoveTo" or command.Name == "MoveBy":
                self.__move_to(command.Direction, command.Speed, self._target_pose.Coord)
            elif (command.Name == "RotateTo" or command.Name == "RotateBy"
                  or command.Name == "RotateTowards"):
                self.__rotate_to(command.Speed, self._target_pose.Angle)
        elif signal.Name == "ResetPose":
            self._robot_pose = self._robot.get_virtual().get_pose()
            self._previous_pose = self._robot_pose
        elif signal.Name == "Pose":
            self.__update_pose(signal.Data)
        elif signal.Name == "Motion" and signal.Data.Control == "DeadReckoning":
            self.__react_motion_deadreckoning(signal.Data)
    def __update_pose(self, pose):
        self._previous_pose = self._robot_pose
        self._robot_pose = pose
        if self._target_pose is not None and self.__reached_pose():
            self.__finish_motion(True)
    def __finish_motion(self, whether_broadcast):
        if whether_broadcast:
            self.broadcast(Signal("Moved", self.get_name(), self._robot.get_name(),
                                  (self._last_command, self._robot_pose)))
        self._target_pose = None
        self._robot.move(0)
    def __react_motion_deadreckoning(self, command):
        if command.Name == "MoveTo":
            self._last_command = command
            self.__move_to(command.Direction, command.Speed, to_vector(*command.Data))
        elif command.Name == "MoveBy":
            self._last_command = command
            current_direction = direction_vector(self._robot_pose.Angle)
            current_direction = current_direction * command.Direction
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
            target_coords = to_vector(*command.Data)
            offset = vector_to_tuple(target_coords - self._robot_pose.Coord)
            target_angle = np.arctan2(offset[1], offset[0])
            if command.Direction == -1:
                target_angle = normalize_angle(target_angle + np.pi)
            self.__rotate_to(command.Speed, target_angle)
    def __reached_pose(self):
        if self._target_pose.Angle is not None:
            target = self._target_pose.Angle
            current = self._robot_pose.Angle
            previous = self._previous_pose.Angle
            return within(previous, current, target)
        else:
            target = vector_to_tuple(self._target_pose.Coord)
            current = vector_to_tuple(self._robot_pose.Coord)
            previous = vector_to_tuple(self._previous_pose.Coord)
            within_x = target[0] is None or within(previous[0], current[0], target[0])
            within_y = target[1] is None or within(previous[1], current[1], target[1])
            return within_x and within_y
    def __move_to(self, direction, speed, target):
        self._target_pose = Pose(target, None)
        if self.__reached_pose():
            self.__finish_motion(True)
        else:
            self._robot.move(abs(speed) * direction)
    def __rotate_to(self, speed, target):
        delta = normalize_angle(normalize_angle(target) - normalize_angle(self._robot_pose.Angle))
        self._target_pose = Pose(to_vector(None, None), delta + self._robot_pose.Angle)
        if self.__reached_pose():
            self.__finish_motion(True)
        else:
            self._robot.rotate(int(speed * np.sign(delta)))

class SimplePrimitivePlanner(Reactor, Broadcaster):
    """Sequentially broadcasts motion commands to a PrimitiveController.

    Signals Received:
        Will react to any Signal of correct name whose Namespace matches the name
        of its robot.
        Start: instructs the planner to start sending motion commands.
        Reset: resets the planner to its initial state and discards all waiting Signals.

    Signals Sent:
        Motion: broadcasts a motion command.
        Stop: broadcasts a signal to stop the controller when resetting the planner.
    """
    def __init__(self, name, robot):
        super(SimplePrimitivePlanner, self).__init__(name)
        self._robot = robot
        self.__command_generator = self._generate_commands()
        next(self.__command_generator)

    # Implementation of parent abstract methods
    def _react(self, signal):
        if not signal.Namespace == self._robot.get_name():
            return
        if signal.Name == "Start":
            self._broadcast_next_command()
        elif signal.Name == "Moved":
            self._broadcast_next_command()
        elif signal.Name == "Reset":
            self.broadcast(Signal("Stop", self.get_name(), self._robot.get_name(), None))
            self.__command_generator.send(False)
            self.clear()
    def _broadcast_next_command(self):
        command = self.__command_generator.send(True)
        self.broadcast(Signal("Motion", self.get_name(), self._robot.get_name(), command))

    # Abstract methods
    def _generate_commands(self):
        """A generator that yields the next motion command.

        Sending:
            Send True into _generate_commands to get the next motion command.
            Send False into _generate_commands to reset the generator.

        Yielding:
            The next motion command for the PrimitiveController.
        """
        yield Motion(None, None, None, None, None)
