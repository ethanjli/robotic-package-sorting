"""Script to test basic message-passing with GUIReactor.
Continuously reads out sensor values monitored by Monitor."""
import time
import Tkinter as tk
import tkMessageBox

from components.messaging import Broadcaster
from components.concurrency import GUIReactor
from hamster.comm_usb import RobotComm

def _ordinal(n):
    # Algorithm from Gareth's solution at:
    # http://codegolf.stackexchange.com/questions/4707/outputting-ordinal-numbers-1st-2nd-3rd
    k = n % 10
    return "{}{}".format(n, "tsnrhtdd"[(n / 10 % 10 != 1) * (k < 4) * k::4])

class RobotApp(GUIReactor, Broadcaster):
    """Shows a simple window with a hello world message and a quit button."""
    def __init__(self, num_robots=1):
        super(RobotApp, self).__init__()
        self.__comm = RobotComm(num_robots, -50)
        self.__num_robots = num_robots
        self._robots = []
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
        tk.Button(parent, name="connect", text="Connect",
                  command=self.__connect_all).pack(side=tk.LEFT, fill=tk.Y)
        tk.Button(parent, name="quit", text="Quit",
                  command=self.quit).pack(side=tk.LEFT, fill=tk.Y)

    # Connect button callback
    def __connect_all(self):
        self.__parent_frame.nametowidget("connect").config(state=tk.DISABLED)
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
        while len(self.__comm.robotList) <= len(self._robots):
            message = "Please connect the {} robot".format(_ordinal(len(self._robots) + 1))
            if not tkMessageBox.askokcancel("Robot Connection Manager", message):
                return False
            if len(self._robots) == 0 and not self.__comm.start():
                tkMessageBox.showerror("Robot Connection Manager",
                                       "Cannot start the robot connection manager.")
                self.quit()
                return False
        self._robots.append(self.__comm.robotList[len(self._robots)])
        return True

    # Abstract methods
    def _initialize_threads(self):
        """Register Reactors into self._threads and set up message-passing."""
        pass
    def _connect_post(self):
        """Executes after robots are connected."""
        pass

