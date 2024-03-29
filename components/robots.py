"""Classes for management of hamster and virtual robots."""
import time
from collections import namedtuple
from sys import platform

import numpy as np

# Select Hamster communication interface depending on operating system. OS X supports
# BLE over the Hamster communication API, while other systems require use of a USB dongle.
# The USB dongle does not support connection to multiple robots in the v1 distribution
# of the Hamster communication API.
if platform == "darwin":
    from hamster.comm_ble import RobotComm as HamsterComm
else:
    from hamster.comm_usb import RobotComm as HamsterComm

from components.util import rescale, clip, get_interpolator
from components.messaging import Signal
from components.concurrency import InterruptableThread, Reactor
from components.geometry import Pose, MobileFrame, direction_vector, to_vector
from components.geometry import transform_all

_PSD_PORT = 0
_SERVO_PORT = 1

_INSTANT_CENTER_OFFSET = 0.5 # cm
_ROBOT_SIZE = 4 # cm

# Coord should be a numpy array, Angle and Servo should be in radians
VirtualState = namedtuple("VirtualState", ["State", "Data"])

class Robot(object):
    """Proxy for a Hamster robot and/or a simulated version of the robot.
    Public attributes calibrate virtual robot sensors and effectors against real values."""
    def __init__(self, hamster_robot=None, virtual_robot=None):
        self._hamster = hamster_robot
        self._virtual = virtual_robot
        self.move_multiplier = 0.095 # speed multiplier
        self.rotate_multiplier = 0.052 # speed multiplier
        self.floor_black = 10 # the blackest measurable value
        self.floor_white = 90 # the whitest measurable value
        prox_profile = ((24, 16), (28, 14), (34, 12), (48, 10), (50, 8), (60, 6),
                        (75, 4), (85, 2))
        self.prox_profile_interp = get_interpolator(prox_profile, np.nan, 2)
        prox_profile_inverse = tuple((dist, prox) for (prox, dist) in reversed(prox_profile))
        self.prox_profile_inverse_interp = get_interpolator(prox_profile_inverse, 84, np.nan)
        psd_profile = ((64, 28), (72, 26), (74, 24), (77, 22), (86, 20), (95, 18), (104, 16),
                       (117, 14), (132, 12), (156, 10), (186, 8), (194, 6))
        self.psd_profile_interp = get_interpolator(psd_profile, np.nan, 6)
        psd_profile_inverse = tuple((dist, psd) for (psd, dist) in reversed(psd_profile))
        self.psd_profile_inverse_interp = get_interpolator(psd_profile_inverse, 192, np.nan)

    # Initialization
    def init_psd_scanner(self):
        """Initialize the I/O ports to run the PSD scanner."""
        if self._hamster is not None:
            self._hamster.set_io_mode(_PSD_PORT, 0x0)
            self._hamster.set_io_mode(_SERVO_PORT, 0x08)

    # Attribute access
    def is_real(self):
        """Checks whether the Robot corresponds to a robot in the real-world.
        As opposed to a purely simulated one."""
        return self._hamster is not None
    def get_virtual(self):
        """Returns the VirtualRobot."""
        return self._virtual
    def get_name(self):
        """Returns the name of the VirtualRobot, or else the Robot itself as a string."""
        if self._virtual is not None:
            return self._virtual.get_name()
        elif self.is_real():
            return repr(self)

    # Effectors
    def beep(self, note):
        """Set the buzzer to the specified musical note.

        Arguments:
            note: the musical note. 0 is silent, 1 - 88 represents piano keys.
        """
        if self.is_real():
            self._hamster.set_musical_note(note)
    def move(self, speed):
        """Move the robot forwards/backwards at the specified speed.

        Arguments:
            speed: the movement speed. -100 (backwards) to 100 (forwards).
        """
        if self.is_real():
            self._hamster.set_wheel(0, speed)
            self._hamster.set_wheel(1, speed)
        if self._virtual is not None:
            self._virtual.move(self.to_virtual_move_speed(speed))
    def rotate(self, speed):
        """Rotate the robot counterclockwise/clockwise at the specified speed.

        Arguments:
            speed: the movement speed. -100 (clockwise) to 100 (counterclockwise).
        """
        if self.is_real():
            self._hamster.set_wheel(0, -speed)
            self._hamster.set_wheel(1, speed)
        if self._virtual is not None:
            self._virtual.rotate(self.to_virtual_rotate_speed(speed))
    def servo(self, angle):
        """Rotate the PSD scanner's servo to the specified angle in degrees.

        Arguments:
            angle: the target angle. 1 to 180.
        """
        if self.is_real():
            self._hamster.set_port(_SERVO_PORT, angle)
        if self._virtual is not None:
            self._virtual.servo(angle)
    def led(self, left_color, right_color):
        """Set the robot's LEDs."""
        if self.is_real():
            self._hamster.set_led(0, left_color)
            self._hamster.set_led(1, right_color)

    # Sensors
    def get_floor(self):
        """Return the values of the floor sensors.

        Return:
            A 2-tuple of the left and right floor values.
        """
        return (self._hamster.get_floor(0), self._hamster.get_floor(1))
    def get_proximity(self):
        """Return the values of the proximity sensors.

        Return:
            A 2-tuple of the left and right proximity values.
        """
        return (self._hamster.get_proximity(0), self._hamster.get_proximity(1))
    def get_psd(self):
        """Return the values of the PSD sensor.

        Return:
            The PSD value.
        """
        return self._hamster.get_port(_PSD_PORT)

    # Calibration
    def set_wheel_balance(self, wheel_balance):
        """Sets the wheel balance, -128 to 127 inclusive."""
        if self._hamster is not None:
            self._hamster.set_wheel_balance(wheel_balance)
    def to_virtual_move_speed(self, real_move_speed):
        """Calculate the virtual move speed for the given real move speed."""
        return real_move_speed * self.move_multiplier
    def to_virtual_rotate_speed(self, real_rotate_speed):
        """Calculate the virtual rotate speed for the given real rotate speed."""
        return real_rotate_speed * self.rotate_multiplier
    def to_relative_whiteness(self, color):
        """Calculate the relative whiteness within the measurable color scale.
        Output is a value between 0 and 255, where 0 corresponds to the blackest
        measurable value and 255 corresponds to the whitest measurable value."""
        return clip(0, 255, rescale(self.floor_black, self.floor_white, 0, 255, color))
    def to_prox_distance(self, proximity):
        """Calculate the distance from the front of the robot."""
        distance = self.prox_profile_interp(proximity)
        return None if np.isnan(distance) else distance
    def to_prox_ir(self, distance):
        """Calculate the distance from the front of the robot."""
        proximity = self.prox_profile_inverse_interp(distance)
        return None if np.isnan(proximity) else proximity
    def to_psd_distance(self, psd):
        """Calculate the distance from the front of the PSD sensor."""
        distance = self.psd_profile_interp(psd)
        return None if np.isnan(distance) else distance
    def to_psd_ir(self, distance):
        """Calculate the distance from the front of the PSD sensor."""
        psd = self.psd_profile_inverse_interp(distance)
        return None if np.isnan(psd) else psd

class Beeper(Reactor):
    """Beeps. Useful for notifications.

    Signals Received:
        Will react to any Signal named Beep whose Namespace matches the name of its robot.
        Data should be a 2-tuple of the note and its duration.
    """
    def __init__(self, name, robot):
        super(Beeper, self).__init__(name)
        self._robot = robot

    def _react(self, signal):
        if signal.Name == "Beep" and signal.Namespace == self._robot.get_name():
            self._robot.beep(signal.Data[0])
            time.sleep(signal.Data[1])
    def _run_post(self):
        self._robot.beep(0)
class Mover(Reactor):
    """Moves the robot using its wheels.

    Signals Received:
        Will react to any Signal of correct name whose Namespace matches the name
        of its robot.
        Advance: Data should be a positive int of the speed.
        Reverse: Data should be a positive int of the speed.
        Stop: Data is ignored.
        Rotate Left: Data should be a positive int of the speed.
        Rotate Right: Data should be a positive int of the speed.
    """
    def __init__(self, name, robot):
        super(Mover, self).__init__(name)
        self._robot = robot

    def _react(self, signal):
        if not signal.Namespace == self._robot.get_name():
            return
        if signal.Name == "Stop":
            self._stop()
        elif signal.Name == "Advance":
            self._robot.move(signal.Data)
        elif signal.Name == "Reverse":
            self._robot.move(-signal.Data)
        elif signal.Name == "Rotate Left":
            self._robot.rotate(signal.Data)
        elif signal.Name == "Rotate Right":
            self._robot.rotate(-signal.Data)
    def _run_pre(self):
        self._stop()
    def _run_post(self):
        self._stop()

    def _stop(self):
        self._robot.move(0)

def centroid_to_instant_center(centroid_pose):
    """Converts a pose with Coord as center of mass to Coord as center of rotation."""
    return Pose(centroid_pose.Coord
                - direction_vector(centroid_pose.Angle) * _INSTANT_CENTER_OFFSET,
                centroid_pose.Angle)

class VirtualScanner(MobileFrame):
    """Models a PSD scanner mounted on a VirtualRobot."""
    def __init__(self, angle=0):
        super(VirtualScanner, self).__init__()
        self.__initial_servo_angle = angle
        self.servo_angle = angle

    # Implementation of parent abstract methods
    def get_pose(self):
        return Pose(to_vector(0, 0), self.servo_angle - 0.5 * np.pi)
    def reset_pose(self):
        self.servo_angle = self.__initial_servo_angle

    # PSD sensor
    def get_psd_coords(self):
        """Returns the coordinates of the Scanner's PSD sensor as column vectors.
        PSD sensor is given as a line segment from the rear end of the sensor to the
        front end of the sensor; then the ray directed out of the sensor is given by
        the first tuple member as a point and the difference between the second and first
        tuple members as the direction.
        """
        return (to_vector(2.5, -1), to_vector(3, -1))
    def get_psd_distance_coords(self, distance):
        """Returns the coordinates of the obstacle from the PSD sensor as a column vector."""
        return None if distance is None else self.get_psd_coords()[0] + to_vector(distance, 0)

class VirtualRobot(InterruptableThread, MobileFrame):
    """Virtual robot to simulate a hamster robot.

    Signals Broadcast:
        Pose: broadcasts the current pose of the robot. Angle is not normalized.
        ScannerPose: broadcasts the current pose of the robot's Scanner.
        ResetPose: broadcasts the current pose of the robot as it is (re)initialized.
    """
    def __init__(self, name, update_interval=0.02,
                 pose=centroid_to_instant_center(Pose(to_vector(0, 0), 0)),
                 servo_angle=0):
        # The Coord of the input pose specifies the centroid of the robot
        super(VirtualRobot, self).__init__(name)
        self.__initial_pose = pose
        self._pose_coord = pose.Coord
        self._pose_angle = pose.Angle
        self._scanner = VirtualScanner(servo_angle)
        self.__update_interval = update_interval
        self.__update_time = time.time()
        self._state = VirtualState("Stopped", None)

    def move(self, speed):
        """Move the robot forwards/backwards at the specified speed.

        Arguments:
            speed: the movement speed. -100 (backwards) to 100 (forwards).
        """
        self.__update_time = time.time()
        if speed == 0:
            self._state = VirtualState("Stopped", None)
        else:
            self._state = VirtualState("Moving", speed)
    def rotate(self, speed):
        """Rotate the robot counterclockwise/clockwise at the specified speed.

        Arguments:
            speed: the movement speed. -100 (clockwise) to 100 (counterclockwise).
        """
        self.__update_time = time.time()
        if speed == 0:
            self._state = VirtualState("Stopped", None)
        else:
            self._state = VirtualState("Rotating", speed)
    def servo(self, angle):
        """Rotate the PSD scanner's servo to the specified angle in degrees.

        Arguments:
            angle: the target angle. 1 to 180.
        """
        self._scanner.servo_angle = np.radians(angle)
        self.__broadcast_servo_pose()
    # Chassis
    def get_corners(self):
        """Returns a tuple of the robot chassis's coordinates as column vectors."""
        return (to_vector(-0.75, 2.1), to_vector(-0.75, 1.7), to_vector(-1.5, 1.7),
                to_vector(-1.5, -1.7), to_vector(-0.75, -1.7), to_vector(-0.75, -2.1),
                to_vector(2.5, -2.1), to_vector(3, -1.5), to_vector(3.4, -1), to_vector(2.5, -1.4),
                to_vector(2.5, 1.4), to_vector(3.4, 1), to_vector(3, 1.5), to_vector(2.5, 2.1))
    # Floor sensors
    def get_floor_centers(self):
        """Returns the centers of the robot's left & right floor sensors as column vectors."""
        return (to_vector(1.75, 0.85), to_vector(1.75, -0.85))
    def get_left_floor_corners(self):
        """Returns a tuple of the corners of the robot's left floor sensor as column vectors."""
        return (to_vector(1.6, 0.6), to_vector(1.6, 1.1),
                to_vector(1.9, 1.1), to_vector(1.9, 0.6))
    def get_right_floor_corners(self):
        """Returns a tuple of the corners of the robot's right floor sensor as column vectors."""
        return (to_vector(1.6, -0.6), to_vector(1.6, -1.1),
                to_vector(1.9, -1.1), to_vector(1.9, -0.6))
    # Proximity IR sensors
    def get_proximity_coords(self):
        """Returns the locations of the robot's left & right proximity sensors as column vectors."""
        return (to_vector(2.5, 1.7), to_vector(2.5, -1.7))
    def get_proximity_distance_coords(self, left_distance, right_distance):
        """Returns the coordinates of the obstacles from the left and right proximity distances."""
        sensor_coords = self.get_proximity_coords()
        left_coord = (None if left_distance is None
                      else sensor_coords[0] + to_vector(left_distance, 0))
        right_coord = (None if right_distance is None
                       else sensor_coords[1] + to_vector(right_distance, 0))
        return (left_coord, right_coord)
    # PSD scanner
    def get_scanner(self):
        """Returns the robot's VirtualScanner."""
        return self._scanner
    # Manual pose adjustment
    def set_pose(self, new_pose):
        """Manually adjust the pose.
        Behaves like a reset_pose call in that it broadcasts a Signal indicating
        a potentially large change in the pose.
        """
        self._pose_coord = new_pose.Coord
        self._pose_angle = new_pose.Angle
        self.broadcast(Signal("SetPose", self.get_name(), self.get_name(), self.get_pose()))

    # Implementation of parent abstract methods
    def _run(self):
        while not self.will_quit():
            curr_time = time.time()
            delta_time = curr_time - self.__update_time
            self.__update_time = curr_time
            if self._state.State == "Moving":
                speed = self._state.Data
                direction = direction_vector(self._pose_angle)
                self._pose_coord = self._pose_coord + speed * delta_time * direction
                self.__broadcast_pose()
            elif self._state.State == "Rotating":
                speed = self._state.Data
                self._pose_angle = self._pose_angle + speed * delta_time
                self.__broadcast_pose()
            time.sleep(self.__update_interval)
    def __broadcast_pose(self):
        self.broadcast(Signal("Pose", self.get_name(), self.get_name(), self.get_pose()))
    def __broadcast_servo_pose(self):
        self.broadcast(Signal("ScannerPose", self.get_name(), self.get_name(),
                              self._scanner.get_pose()))
    def get_pose(self):
        return Pose(self._pose_coord, self._pose_angle)
    def reset_pose(self):
        self._pose_coord = self.__initial_pose.Coord
        self._pose_angle = self.__initial_pose.Angle
        self._scanner.reset_pose()
        self.broadcast(Signal("ResetPose", self.get_name(), self.get_name(), self.get_pose()))
        self.broadcast(Signal("ScannerPose", self.get_name(), self.get_name(),
                              self._scanner.get_pose()))

