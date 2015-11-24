"""Script to calibrate a robot."""
import sys
import Tkinter as tk
import ttk

from components.messaging import Signal
from components.robots import VirtualRobot
from components.sensors import SimpleMonitor, FilteringMonitor
from components.world import VirtualWorld, Wall
from components.app import Simulator

class GUICalibrate(Simulator):
    """Reads out robot sensor values."""
    def __init__(self, name="Sensors GUI", update_interval=10):
        super(GUICalibrate, self).__init__(name, update_interval, 1)
        self._initialize_widgets()
        self._initialize_world(2)

    # Implementing abstract methods
    def _react(self, signal):
        pass
    def _initialize_widgets(self):
        app_frame = ttk.LabelFrame(self._root, name="appFrame",
                                   borderwidth=2, relief="ridge",
                                   text="App")
        app_frame.pack(fill="x")
        self._initialize_robotapp_widgets(app_frame)

        simulator_frame = ttk.LabelFrame(self._root, name="simulatorFrame",
                                         borderwidth=2, relief="ridge",
                                         text="Simulator")
        simulator_frame.pack(fill="both", expand="yes")
        self._initialize_simulator_widgets(simulator_frame, [-40, -40, 40, 40], 10)
    def _initialize_threads(self):
        virtual = self._robots[0].get_virtual()
        virtual.register("Position", self)
        self._threads[virtual.get_name()] = virtual
    def _connect_post(self):
        self._reset_simulator()
    def _generate_virtual_robots(self):
        for i in range(0, self._num_robots):
            yield VirtualRobot("Virtual {}".format(i))
    def _populate_world(self):
        self._world.add_wall(Wall(0, 0, 8))

def main():
    """Runs test."""
    gui = GUICalibrate()
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
