"""Classes for management of hamster and virtual robots."""
import time
import tkMessageBox
import ttk

from components.util import ordinal
from components.messaging import Receiver, Broadcaster
from components.concurrency import GUIReactor
from hamster.comm_usb import RobotComm

MIN_RSSI = -50

_PSD_PORT = 0
_SERVO_PORT = 1

class Robot(object):
    """Proxy for a Hamster robot and/or a simulated version of the robot."""
    def __init__(self, hamster_robot):
        self._hamster = hamster_robot

    # Initialization
    def init_psd_scanner(self):
        """Initialize the I/O ports to run the PSD scanner."""
        self._hamster.set_io_mode(_PSD_PORT, 0x0)
        self._hamster.set_io_mode(_SERVO_PORT, 0x08)

    # Effectors
    def beep(self, note):
        """Set the buzzer to the specified musical note.

        Arguments:
            note: the musical note. 0 is silent, 1 - 88 represents piano keys.
        """
        self._hamster.set_musical_note(note)
    def move(self, speed):
        """Move the robot forwards/backwards at the specified speed.

        Arguments:
            speed: the movement speed. -100 (backwards) to 100 (forwards).
        """
        self._hamster.set_wheel(0, speed)
        self._hamster.set_wheel(1, speed)
    def rotate(self, speed):
        """Rotate the robot counterclockwise/clockwise at the specified speed.

        Arguments:
            speed: the movement speed. -100 (clockwise) to 100 (counterclockwise).
        """
        self._hamster.set_wheel(0, -speed)
        self._hamster.set_wheel(1, speed)
    def servo(self, angle):
        """Rotate the PSD scanner's servo to the specified angle.

        Arguments:
            angle: the target angle. 1 to 180.
        """
        self._hamster.set_port(_SERVO_PORT, angle)

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

class RobotApp(GUIReactor, Broadcaster):
    """Shows a simple window with a hello world message and a quit button."""
    def __init__(self, name="App", update_interval=10, num_robots=1):
        super(RobotApp, self).__init__(name, update_interval)
        self.__hamster_comm = None
        self.__num_robots = num_robots
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
                return False
            while self.__hamster_comm is None:
                if not self.__start_hamster_comm():
                    return False
        next_hamster = self.__hamster_comm.robotList[len(self._robots)]
        self._robots.append(Robot(next_hamster))
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

    # Abstract methods
    def _initialize_threads(self):
        """Register Reactors into self._threads and set up message-passing."""
        pass
    def _connect_post(self):
        """Executes after robots are connected."""
        pass

