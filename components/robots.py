"""Classes for management of hamster and virtual robots."""
import time
from collections import namedtuple
import Tkinter as tk
import tkMessageBox
import ttk

import numpy as np

from components.util import ordinal
from components.messaging import Signal, Broadcaster
from components.concurrency import InterruptableThread, Reactor, GUIReactor
from hamster.comm_ble import RobotComm

MIN_RSSI = -50

_PSD_PORT = 0
_SERVO_PORT = 1

_INSTANT_CENTER_OFFSET = 0.1 # cm

# Coord should be a numpy array, Angle and Servo should be in radians
Pose = namedtuple("Pose", ["Coord", "Angle", "Servo"])
VirtualState = namedtuple("VirtualState", ["State", "Data"])

class Robot(object):
    """Proxy for a Hamster robot and/or a simulated version of the robot."""
    def __init__(self, hamster_robot=None, virtual_robot=None):
        self._hamster = hamster_robot
        self._virtual = virtual_robot

    # Initialization
    def init_psd_scanner(self):
        """Initialize the I/O ports to run the PSD scanner."""
        if self._hamster is not None:
            self._hamster.set_io_mode(_PSD_PORT, 0x0)
            self._hamster.set_io_mode(_SERVO_PORT, 0x08)

    # Effectors
    def beep(self, note):
        """Set the buzzer to the specified musical note.

        Arguments:
            note: the musical note. 0 is silent, 1 - 88 represents piano keys.
        """
        if self._hamster is not None:
            self._hamster.set_musical_note(note)
    def move(self, speed):
        """Move the robot forwards/backwards at the specified speed.

        Arguments:
            speed: the movement speed. -100 (backwards) to 100 (forwards).
        """
        if self._hamster is not None:
            self._hamster.set_wheel(0, speed)
            self._hamster.set_wheel(1, speed)
        if self._virtual is not None:
            self._virtual.move(speed)
    def rotate(self, speed):
        """Rotate the robot counterclockwise/clockwise at the specified speed.

        Arguments:
            speed: the movement speed. -100 (clockwise) to 100 (counterclockwise).
        """
        if self._hamster is not None:
            self._hamster.set_wheel(0, -speed)
            self._hamster.set_wheel(1, speed)
        if self._virtual is not None:
            self._virtual.rotate(speed)
    def servo(self, angle):
        """Rotate the PSD scanner's servo to the specified angle.

        Arguments:
            angle: the target angle. 1 to 180.
        """
        if self._hamster is not None:
            self._hamster.set_port(_SERVO_PORT, angle)
        if self._virtual is not None:
            self._virtual.servo(angle)

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
class Beeper(Reactor):
    """Beeps. Useful for notifications.

    Signals Received:
        Will react to any Signal, regardless of name or sender. Data should be a
        2-tuple of the note and its duration.
    """
    def __init__(self, name, robot):
        super(Beeper, self).__init__(name)
        self._robot = robot

    def _react(self, signal):
        self._robot.beep(signal.Data[0])
        time.sleep(signal.Data[1])
    def _run_post(self):
        self._robot.beep(0)
class Mover(Reactor):
    """Moves the robot using its wheels.

    Signals Received:
        Will react to any Signal of correct name, regardless of sender.
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
                - angle_to_unit_vector(centroid_pose.Angle) * _INSTANT_CENTER_OFFSET,
                centroid_pose.Angle, centroid_pose.Servo)
def angle_to_unit_vector(angle):
    """Returns the unit vector associated with an angle (ccw from +x)."""
    return np.array([np.cos(angle), np.sin(angle)])

class VirtualRobot(InterruptableThread, Broadcaster):
    """Virtual robot to simulate a hamster robot."""
    def __init__(self, name, update_interval=0.01,
                 pose=centroid_to_instant_center(Pose(np.array([0, 0]), 0, 0))):
        # The Coord of the input pose specifies the centroid of the robot
        super(VirtualRobot, self).__init__(name)
        self._pose_coord = pose.Coord
        self._pose_angle = pose.Angle
        self._pose_servo = pose.Servo
        self.__update_interval = update_interval
        self.__update_time = time.time()
        self._state = VirtualState("Stopped", None)

    def get_pose(self):
        return Pose(self._pose_coord, self._pose_angle, self._pose_servo)

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
        """Rotate the PSD scanner's servo to the specified angle.

        Arguments:
            angle: the target angle. 1 to 180.
        """
        self.__update_time = time.time()
        self._pose_servo = angle

    # Implementation of parent abstract methods
    def _run(self):
        while not self.will_quit():
            curr_time = time.time()
            delta_time = curr_time - self.__update_time
            self.__update_time = curr_time
            if self._state.State == "Moving":
                speed = self._state.Data
                direction = angle_to_unit_vector(self._pose_angle)
                self._pose_coord = self._pose_coord + speed * delta_time * direction
                self.broadcast(Signal("Position", self.get_name(), self.get_pose()))
            elif self._state.State == "Rotating":
                speed = self._state.Data
                self._pose_angle = self._pose_angle + speed * delta_time
                self.broadcast(Signal("Position", self.get_name(), self.get_pose()))
            time.sleep(self.__update_interval)

class RobotApp(GUIReactor, Broadcaster):
    """Shows a simple window with a hello world message and a quit button."""
    def __init__(self, name="App", update_interval=10, num_robots=1):
        super(RobotApp, self).__init__(name, update_interval)
        self.__hamster_comm = None
        self.__num_robots = num_robots
        self.__virtual_robot_generator = self._generate_virtual_robots()
        self._robots = []
        self._threads = {}
        self.__parent_frame = None

    # Implementing abstract methods
    def _run_post(self):
        for _, thread in self._threads.items():
            thread.quit()
        if self.__hamster_comm is None:
            return
        for hamster_robot in self.__hamster_comm.robotList:
            hamster_robot.reset()
        time.sleep(1.0)
        self.__hamster_comm.stop()

    # Utility for subclasses
    def _initialize_robotapp_widgets(self, parent):
        """Add RobotApp widgets into the specified parent widget.

        Widgets:
            connect: a button to connect to the robots.
            quit: a button to exit the application.
        """
        self.__parent_frame = parent
        ttk.Button(parent, name="connect", text="Connect",
                   command=self.__connect_all).pack(side="left", fill="y")
        ttk.Button(parent, name="quit", text="Quit",
                   command=self.quit).pack(side="left", fill="y")

    # Connect button callback
    def __connect_all(self):
        self.__parent_frame.nametowidget("connect").config(state="disabled")
        while len(self._robots) < self.__num_robots:
            if not self.__connect_next():
                self.quit()
                return
        self._initialize_threads()
        for _, thread in self._threads.items():
            thread.start()
        self.__parent_frame.nametowidget("connect").config(text="Connected")
        self._connect_post()
    def __connect_next(self):
        while (self.__hamster_comm is None or
               len(self.__hamster_comm.robotList) <= len(self._robots)):
            message = "Please connect the {} robot".format(ordinal(len(self._robots) + 1))
            if not tkMessageBox.askokcancel("Robot Connection Manager", message):
                return self.__add_virtual_substitute()
            while self.__hamster_comm is None:
                self.__start_hamster_comm()
                if not self.__start_hamster_comm():
                    return False
        next_hamster = self.__hamster_comm.robotList[len(self._robots)]
        self._robots.append(Robot(next_hamster, next(self.__virtual_robot_generator)))
        return True
    def __start_hamster_comm(self):
        self.__hamster_comm = RobotComm(self.__num_robots, MIN_RSSI)
        if not self.__hamster_comm.start():
            self.__hamster_comm = None
            if not tkMessageBox.askretrycancel("Robot Connection Manager",
                                               "Cannot start the robot connection "
                                               "manager. Please try again."):
                self.quit()
                return False
        return True
    def __add_virtual_substitute(self):
        if tkMessageBox.askokcancel("Robot Connection Manager",
                                    "Add virtual robot instead?"):
            self._robots.append(Robot(None, next(self.__virtual_robot_generator)))
            return True
        else:
            return False

    # Abstract methods
    def _initialize_threads(self):
        """Register Reactors into self._threads and set up message-passing."""
        pass
    def _connect_post(self):
        """Executes after robots are connected."""
        pass
    def _generate_virtual_robots(self):
        """A generator to yield the specified number of virtual robots
        The order of virtual robots generated should correspond to the order of
        hamster robots added."""
        for _ in range(0, self.__num_robots):
            yield None

class Simulator(RobotApp):
    """Displays and simulates the virtual world of a robot."""
    def __init__(self, name="Simulator", update_interval=10, num_robots=1):
        super(Simulator, self).__init__(name, update_interval, num_robots)
        self.__parent_frame = None
        self.__canvas = None

    # Utility for subclasses
    def _initialize_simulator_widgets(self, parent, bounds):
        """Add RobotSimulator widgets into the specified parent widget.

        Widgets:
            canvas: canvas displaying the virtual world.
            reset: a button to reset the virtual world to its initial state.
            bounds: a 4-tuple of the min x coord, min y coord, max x coord, and
            max y coord that the canvas should be able to display.
        """
        self.__parent_frame = parent
        self.__canvas = tk.Canvas(self._root, name="canvas", bg="white",
                                  width=(bounds[2] - bounds[0]),
                                  height=(bounds[3] - bounds[1]))
