"""Script to test basic message-passing with GUIReactor.
Continuously reads out sensor values monitored by Monitor."""
import time
import Tkinter as tk

from components.messaging import Broadcaster
from components.concurrency import GUIReactor
from hamster.comm_usb import RobotComm

class RobotApp(GUIReactor, Broadcaster):
    """Shows a simple window with a hello world message and a quit button."""
    def __init__(self, num_robots=1):
        super(RobotApp, self).__init__()
        self.__comm = RobotComm(num_robots, -50)
        self.__num_robots = num_robots
        self._threads = {}
        self.__parent_frame = None

    # Implementing abstract methods
    def _run_post(self):
        for _, thread in self._threads.items():
            thread.quit()
        for robot in self.__comm.robotList:
            robot.reset()
        time.sleep(1.0)
        self.__comm.stop()

    # Utility for subclasses
    def _initialize_robotapp_widgets(self, parent):
        """Add RobotApp widgets into the specified parent widget.

        Widgets:
            connect: a button to connect to the robots.
            quit: a button to exit the application.
        """
        self.__parent_frame = parent
        tk.Label(parent, name="name", text="Robots").pack()
        tk.Button(parent, name="connect", text="Connect",
                  command=self.__connect).pack(fill=tk.X)
        tk.Button(parent, name="quit", text="Quit",
                  command=self.quit).pack(fill=tk.X)

    # Connect button callback
    def __connect(self):
        if not self.__comm.start():
            raise RuntimeError("Robot communication failed to start.")
        self.__parent_frame.nametowidget("connect").config(state=tk.DISABLED)
        self._register_robots(self.__comm.robotList)
        self._initialize_threads()
        self.__start_threads()
        self.__parent_frame.nametowidget("connect").config(text="Connected")
        self._connect_post()
    def __start_threads(self):
        for _, thread in self._threads.items():
            thread.start()

    # Abstract methods
    def _register_robots(self, robot_list):
        """Register robots from the robot list."""
        pass
    def _initialize_threads(self):
        """Register Reactors into self._threads and set up message-passing."""
        pass
    def _connect_post(self):
        """Executes after robots are connected."""
        pass

