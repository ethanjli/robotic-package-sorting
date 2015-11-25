"""Script to calibrate a robot."""
import sys
import Tkinter as tk
import ttk

from components.messaging import Signal
from components.robots import VirtualRobot, Mover
from components.sensors import SimpleMonitor, FilteringMonitor
from components.world import VirtualWorld, Wall
from components.app import Simulator

class GUICalibrate(Simulator):
    """Reads out robot sensor values."""
    def __init__(self, name="Calibration GUI", update_interval=10):
        super(GUICalibrate, self).__init__(name, update_interval, 1)
        self._initialize_widgets()
        self._initialize_world(2)

    # Implementing abstract methods
    def _react(self, signal):
        pass
    def _initialize_widgets(self):
        toolbar_frame = ttk.Frame(self._root)
        toolbar_frame.pack(side="top", fill="x")
        app_frame = ttk.LabelFrame(toolbar_frame, name="appFrame",
                                   borderwidth=2, relief="ridge",
                                   text="App")
        app_frame.pack(side="left")
        self._initialize_robotapp_widgets(app_frame)

        calibrate_frame = ttk.LabelFrame(toolbar_frame, name="calibrateFrame",
                                         borderwidth=2, relief="ridge",
                                         text="Calibrate")
        calibrate_frame.pack(side="left")
        self.__stop_button = ttk.Button(calibrate_frame, name="stop", text="Stop",
                                        command=self._stop, state="disabled")
        self.__stop_button.pack(side="left")
        self.__rotateccw_button = ttk.Button(calibrate_frame, name="rotateccw", text="Rotate CCW",
                                             command=self._rotateccw, state="disabled")
        self.__rotateccw_button.pack(side="left")
        self.__rotatecw_button = ttk.Button(calibrate_frame, name="rotatecw", text="Rotate CW",
                                            command=self._rotatecw, state="disabled")
        self.__rotatecw_button.pack(side="left")

        simulator_frame = ttk.LabelFrame(self._root, name="simulatorFrame",
                                         borderwidth=2, relief="ridge",
                                         text="Simulator")
        simulator_frame.pack(fill="both", expand="yes")
        self._initialize_simulator_widgets(simulator_frame, [-40, -40, 40, 40], 10)
    def _initialize_threads(self):
        self._add_virtual_world_threads()
        mover = Mover("Mover", self._robots[0])
        self.register("Advance", mover)
        self.register("Reverse", mover)
        self.register("Rotate Left", mover)
        self.register("Rotate Right", mover)
        self.register("Stop", mover)
        self._add_thread(mover)
    def _connect_post(self):
        self._add_robots()
        self._change_reset_button("Reset")
        self.__stop_button.config(state="normal")
        self.__rotateccw_button.config(state="normal")
        self.__rotatecw_button.config(state="normal")
    def _generate_virtual_robots(self):
        for i in range(0, self._num_robots):
            yield VirtualRobot("Virtual {}".format(i))
    def _populate_world(self):
        self._world.add_wall(Wall(0, 0, 8))

    # Stop button callback
    def _stop(self):
        self.broadcast(Signal("Stop", self.get_name(), self._robots[0].get_name(), None))

    # Rotate button callbacks
    def _rotateccw(self):
        self.broadcast(Signal("Rotate Left", self.get_name(), self._robots[0].get_name(), 20))
    def _rotatecw(self):
        self.broadcast(Signal("Rotate Right", self.get_name(), self._robots[0].get_name(), 20))

def main():
    """Runs test."""
    gui = GUICalibrate()
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
