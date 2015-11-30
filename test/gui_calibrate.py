"""Script to calibrate a robot."""
import sys
import Tkinter as tk
import ttk

import numpy as np

from components.messaging import Signal
from components.robots import VirtualRobot
from components.sensors import SimpleMonitor, FilteringMonitor
from components.world import VirtualWorld, Wall
from components.control import Motion, PrimitiveController
from components.app import Simulator

class GUICalibrate(Simulator):
    """Reads out robot sensor values."""
    def __init__(self, name="Calibration GUI", update_interval=10):
        super(GUICalibrate, self).__init__(name, update_interval, 1)
        self._initialize_widgets()
        self._initialize_world(2)

    # Implementing parent abstract methods
    def _react_simulator(self, signal):
        if signal.Name == "Moved":
            self.__set_motion_buttons_state("normal")
            self.__pauseresume_button.config(state="disabled", text="Pause")
    def _initialize_widgets(self):
        toolbar_frame = ttk.Frame(self._root, name="toolbarFrame")
        toolbar_frame.pack(side="top", fill="x")
        app_frame = ttk.LabelFrame(toolbar_frame, name="appFrame",
                                   borderwidth=2, relief="ridge", text="App")
        app_frame.pack(side="left", fill="y")
        self._initialize_robotapp_widgets(app_frame)

        # Calibration toolbar
        self.__calibrate_frame = ttk.LabelFrame(toolbar_frame, name="calibrateFrame",
                                                borderwidth=2, relief="ridge",
                                                text="Calibrate")
        self.__calibrate_frame.pack(side="left")
        # Movement multipliers
        multipliers_frame = ttk.Frame(self.__calibrate_frame, name="multipliersFrame")
        multipliers_frame.pack(side="left", fill="y")
        self.__move_multiplier = tk.StringVar()
        self.__move_multiplier.set("0.095")
        self.__move_multiplier.trace("w", self._move_multiplier)
        move_multiplier_frame = ttk.LabelFrame(multipliers_frame, name="move",
                                               borderwidth=2, relief="ridge",
                                               text="Move Multiplier")
        move_multiplier_frame.pack(side="top", fill="x")
        ttk.Combobox(move_multiplier_frame, name="multipliers",
                     textvariable=self.__move_multiplier, state="disabled",
                     values=[str(0.005 * i) for i in range(2, 41)]).pack()
        self.__rotate_multiplier = tk.StringVar()
        self.__rotate_multiplier.set("0.052")
        self.__rotate_multiplier.trace("w", self._rotate_multiplier)
        rotate_multiplier_frame = ttk.LabelFrame(multipliers_frame, name="rotate",
                                                 borderwidth=2, relief="ridge",
                                                 text="Rotate Multiplier")
        rotate_multiplier_frame.pack(side="bottom", fill="y")
        ttk.Combobox(rotate_multiplier_frame, name="multipliers",
                     textvariable=self.__rotate_multiplier, state="disabled",
                     values=[str(0.001 * i) for i in range(40, 81)]).pack()
        # Commands
        commands_frame = ttk.LabelFrame(self.__calibrate_frame, name="commandsFrame",
                                        borderwidth=2, relief="ridge", text="Commands")
        commands_frame.pack(side="right", fill="y")
        # Stop button
        stop_frame = ttk.Frame(commands_frame, name="stopFrame")
        stop_frame.pack(side="left", fill="y")
        self.__stop_button = ttk.Button(stop_frame, name="stop", text="Stop",
                                        command=self._stop, state="disabled")
        self.__stop_button.pack(side="top", fill="x")
        self.__pauseresume_button = ttk.Button(stop_frame, name="pauseresume", text="Pause",
                                               command=self._pauseresume, state="disabled")
        self.__pauseresume_button.pack(side="top", fill="x")
        # Rotate buttons
        rotate_frame = ttk.Frame(commands_frame, name="rotateFrame")
        rotate_frame.pack(side="left", fill="y")
        self.__rotate90_button = ttk.Button(rotate_frame, name="rotate90",
                                            text="Rotate 90 deg", command=self._rotate90,
                                            state="disabled")
        self.__rotate90_button.pack(side="top", fill="x")
        self.__rotate_90_button = ttk.Button(rotate_frame, name="rotate-90",
                                             text="Rotate -90 deg", command=self._rotate_90,
                                             state="disabled")
        self.__rotate_90_button.pack(side="top", fill="x")
        # Move buttons
        move_frame = ttk.Frame(commands_frame, name="moveFrame")
        move_frame.pack(side="left", fill="y")
        self.__move4_button = ttk.Button(move_frame, name="move4",
                                         text="Move 4", command=self._move4,
                                         state="disabled")
        self.__move4_button.pack(side="top", fill="x")
        self.__move_4_button = ttk.Button(move_frame, name="move-4",
                                          text="Move -4", command=self._move_4,
                                          state="disabled")
        self.__move_4_button.pack(side="top", fill="x")

        simulator_frame = ttk.LabelFrame(self._root, name="simulatorFrame",
                                         borderwidth=2, relief="ridge",
                                         text="Simulator")
        simulator_frame.pack(fill="both", expand="yes")
        self._initialize_simulator_widgets(simulator_frame, [-40, -40, 40, 40], 10)
    def _initialize_threads(self):
        self._add_virtual_world_threads()

        controller = PrimitiveController("MotionController", self._robots[0])
        controller.register("Moved", self)
        self.register("Motion", controller)
        self.register("Stop", controller)
        self.register("Pause", controller)
        self.register("Resume", controller)
        self._add_thread(controller)
    def _connect_post(self):
        self._add_robots()
        self._change_reset_button("Reset")
        movement_multipliers = self.__calibrate_frame.nametowidget("""multipliersFrame.move"""
                                                                   """.multipliers""")
        movement_multipliers.config(state="normal")
        self._move_multiplier(None, None, "w")
        rotate_multipliers = self.__calibrate_frame.nametowidget("""multipliersFrame.rotate"""
                                                                 """.multipliers""")
        rotate_multipliers.config(state="normal")
        self._rotate_multiplier(None, None, "w")
        self.__set_motion_buttons_state("normal")
    def __set_motion_buttons_state(self, new_state):
        self.__rotate90_button.config(state=new_state)
        self.__rotate_90_button.config(state=new_state)
        self.__move4_button.config(state=new_state)
        self.__move_4_button.config(state=new_state)
    def _generate_virtual_robots(self):
        for i in range(0, self._num_robots):
            yield VirtualRobot("Virtual {}".format(i))
    def _populate_world(self):
        self._world.add_wall(Wall(0, 0, 8))

    # Multiplier dropdown callbacks
    def _move_multiplier(self, _, dummy, operation):
        robot = self._robots[0].get_virtual()
        if operation == "w":
            robot.move_multiplier = float(self.__move_multiplier.get())
    def _rotate_multiplier(self, _, dummy, operation):
        robot = self._robots[0].get_virtual()
        if operation == "w":
            robot.rotate_multiplier = float(self.__rotate_multiplier.get())

    def __broadcast_motion_command(self, command):
        self.broadcast(Signal("Motion", self.get_name(), self._robots[0].get_name(), command))
        self.__set_motion_buttons_state("disabled")
        self.__pauseresume_button.config(state="normal", text="Pause")
        self.__stop_button.config(state="normal")

    # Stop button callbacks
    def _stop(self):
        self.broadcast(Signal("Stop", self.get_name(), self._robots[0].get_name(), None))
        self.__set_motion_buttons_state("normal")
        self.__stop_button.config(state="disabled")
    def _pauseresume(self):
        if self.__pauseresume_button.cget("text") == "Pause":
            self.broadcast(Signal("Pause", self.get_name(), self._robots[0].get_name(), None))
            self.__pauseresume_button.config(text="Resume")
        elif self.__pauseresume_button.cget("text") == "Resume":
            self.broadcast(Signal("Resume", self.get_name(), self._robots[0].get_name(), None))
            self.__pauseresume_button.config(text="Pause")

    # Rotate button callbacks
    def _rotate90(self):
        command = Motion("RotateBy", "DeadReckoning", None, 20, 0.5 * np.pi)
        self.__broadcast_motion_command(command)
    def _rotate_90(self):
        command = Motion("RotateBy", "DeadReckoning", None, 20, -0.5 * np.pi)
        self.__broadcast_motion_command(command)

    # Move button callbacks
    def _move4(self):
        command = Motion("MoveBy", "DeadReckoning", "Forwards", 20, 4)
        self.__broadcast_motion_command(command)
    def _move_4(self):
        command = Motion("MoveBy", "DeadReckoning", "Backwards", 20, 4)
        self.__broadcast_motion_command(command)

def main():
    """Runs test."""
    gui = GUICalibrate()
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
