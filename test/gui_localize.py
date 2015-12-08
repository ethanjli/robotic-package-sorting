"""Script to localize a robot."""
import sys
import Tkinter as tk
import ttk

import numpy as np

from components.messaging import Signal
from components.geometry import Pose, to_vector
from components.util import clip
from components.robots import VirtualRobot, centroid_to_instant_center
from components.sensors import FilteringMonitor, VirtualMonitor
from components.world import Border, Wall, Package
from components.control import Motion, PrimitiveController, SimplePrimitivePlanner
from components.app import Simulator

class SquarePlanner(SimplePrimitivePlanner):
    """Plans a square motion path for the robot."""
    def _generate_commands(self):
        motions = [
            Motion("RotateTowards", "DeadReckoning", 1, 20, (3.5, 0)),
            Motion("MoveTo", "DeadReckoning", 1, 20, (3.5, None)),
            Motion("RotateTowards", "DeadReckoning", 1, 20, (3.5, 4)),
            Motion("MoveUntil", "SensorDistance", 1, 20,
                   lambda prox_left, prox_right, psd: prox_left <= 4 and prox_right <= 4),
            Motion("RotateTowards", "DeadReckoning", 1, 20, (-0.5, 4)),
            Motion("MoveTo", "DeadReckoning", 1, 20, (-0.5, None)),
            Motion("RotateTowards", "DeadReckoning", 1, 20, (-0.5, 0)),
            Motion("MoveTo", "DeadReckoning", 1, 20, (None, 0)),
            None
        ]
        while True:
            index = 0
            resetting = False
            while not resetting:
                resetting = not (yield motions[index])
                index = clip(0, len(motions) - 1, index + 1)

class GUILocalize(Simulator):
    """Localizes the robot."""
    def __init__(self, name="Localization GUI", update_interval=10):
        super(GUILocalize, self).__init__(name, update_interval, 1)
        self._initialize_widgets()
        self._initialize_world(2)

    # Implementing parent abstract methods
    def _react_simulator(self, signal):
        if signal.Name == "Moved":
            self.__set_motion_buttons_state("normal")
            self.__pauseresume_button.config(state="disabled", text="Pause")
        elif signal.Name == "Motion":
            self.__set_motion_buttons_state("disabled")
    def _initialize_widgets(self):
        toolbar_frame = ttk.Frame(self._root, name="toolbarFrame")
        toolbar_frame.pack(side="top", fill="x")
        app_frame = ttk.LabelFrame(toolbar_frame, name="appFrame",
                                   borderwidth=2, relief="ridge", text="App")
        app_frame.pack(side="left", fill="y")
        self._initialize_robotapp_widgets(app_frame)

        # Localization toolbar
        self.__localize_frame = ttk.LabelFrame(toolbar_frame, name="localizeFrame",
                                               borderwidth=2, relief="ridge",
                                               text="Localize")
        self.__localize_frame.pack(side="left")
        # Commands
        commands_frame = ttk.LabelFrame(self.__localize_frame, name="commandsFrame",
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
        # Localize buttons
        localize_frame = ttk.Frame(commands_frame, name="localizerFrame")
        localize_frame.pack(side="left", fill="y")
        self.__prox_button = ttk.Button(localize_frame, name="proxLocalize",
                                        text="Prox Localize", command=self._localize_prox,
                                        state="disabled")
        self.__prox_button.pack(side="top", fill="x")
        self.__psd_button = ttk.Button(localize_frame, name="psdLocalize",
                                       text="PSD Localize", command=self._localize_psd,
                                       state="disabled")
        self.__psd_button.pack(side="top", fill="x")

        simulator_frame = ttk.LabelFrame(self._root, name="simulatorFrame",
                                         borderwidth=2, relief="ridge",
                                         text="Simulator")
        simulator_frame.pack(fill="both", expand="yes")
        self._initialize_simulator_widgets(simulator_frame, [-40, -40, 40, 40], 10)
    def _initialize_threads(self):
        self._add_virtual_world_threads()
        self.register("LocalizeProx", self._world)
        self.register("LocalizePSD", self._world)

        if self._robots[0].is_real():
            monitor = FilteringMonitor("Monitor 0", self._robots[0])
        else:
            monitor = VirtualMonitor("Monitor 0", self._robots[0], self._world)
        monitor.register("Floor", self._world)
        monitor.register("Proximity", self._world)
        monitor.register("PSD", self._world)
        self._add_thread(monitor)

        controller = PrimitiveController("MotionController", self._robots[0], monitor)
        controller.register("Moved", self)
        self.register("Motion", controller)
        self.register("Stop", controller)
        self.register("Pause", controller)
        self.register("Resume", controller)
        self._add_thread(controller)

        planner = SquarePlanner("SquarePlanner", self._robots[0])
        planner.register("Motion", controller)
        planner.register("Motion", self)
        planner.register("Stop", controller)
        controller.register("Moved", planner)
        self.register("Start", planner)
        self.register("Reset", planner)
        self._add_thread(planner)

        # Calibration
        self._robots[0].move_multiplier = 0.12
        self._robots[0].rotate_multiplier = 0.065
        self._robots[0].set_wheel_balance(8)
    def _connect_post(self):
        self._change_reset_button("Reset")
        self._enable_start_button()
        self.__set_motion_buttons_state("normal")
        self.__set_localize_buttons_state("normal")
    def __set_motion_buttons_state(self, new_state):
        self.__rotate90_button.config(state=new_state)
        self.__rotate_90_button.config(state=new_state)
        self.__move4_button.config(state=new_state)
        self.__move_4_button.config(state=new_state)
    def __set_localize_buttons_state(self, new_state):
        self.__prox_button.config(state=new_state)
        self.__psd_button.config(state=new_state)
    def _generate_virtual_robots(self):
        for i in range(0, self._num_robots):
            yield VirtualRobot("Virtual {}".format(i),
                               pose=centroid_to_instant_center(Pose(to_vector(-2, -2), 0)),
                               servo_angle=(0.5 * np.pi))
    def _populate_world(self):
        self._world.add_wall(Wall(0, center_y=12, x_length=20))
        self._world.add_wall(Wall(1, center_x=12, x_length=4, y_length=20))
        self._world.add_wall(Wall(2, center_y=-12, x_length=20))
        self._world.add_wall(Wall(3, center_x=-12, x_length=4, y_length=20))
    def _start_simulator(self):
        self.__stop_button.config(state="disabled")
        self.__set_motion_buttons_state("disabled")
        self.__set_localize_buttons_state("disabled")
        self.broadcast(Signal("Start", self.get_name(), self._robots[0].get_name(), None))
    def _pause_simulator(self):
        self.broadcast(Signal("Pause", self.get_name(), self._robots[0].get_name(), None))
        self.__set_motion_buttons_state("normal")
        self.__set_localize_buttons_state("normal")
    def _resume_simulator(self):
        self.broadcast(Signal("Resume", self.get_name(), self._robots[0].get_name(), None))
        self.__set_motion_buttons_state("disabled")
        self.__set_localize_buttons_state("disabled")
    def _reset_simulator_post(self):
        self.broadcast(Signal("Reset", self.get_name(), self._robots[0].get_name(), None))
        self.__set_motion_buttons_state("normal")
        self.__set_localize_buttons_state("normal")

    def __broadcast_motion_command(self, command):
        self.broadcast(Signal("Motion", self.get_name(), self._robots[0].get_name(), command))
        self.__set_motion_buttons_state("disabled")
        self.__set_localize_buttons_state("disabled")
        self.__pauseresume_button.config(state="normal", text="Pause")
        self.__stop_button.config(state="normal")

    # Stop button callbacks
    def _stop(self):
        self.broadcast(Signal("Stop", self.get_name(), self._robots[0].get_name(), None))
        self.__set_motion_buttons_state("normal")
        self.__set_localize_buttons_state("normal")
        self.__stop_button.config(state="disabled")
    def _pauseresume(self):
        state = self.__pauseresume_button.cget("text")
        if state == "Pause":
            self.broadcast(Signal("Pause", self.get_name(), self._robots[0].get_name(), None))
            self.__pauseresume_button.config(text="Resume")
        elif state == "Resume":
            self.broadcast(Signal("Resume", self.get_name(), self._robots[0].get_name(), None))
            self.__pauseresume_button.config(text="Pause")

    # Rotate button callbacks
    def _rotate90(self):
        command = Motion("RotateBy", "DeadReckoning", None, 10, 0.5 * np.pi)
        self.__broadcast_motion_command(command)
    def _rotate_90(self):
        command = Motion("RotateBy", "DeadReckoning", None, 10, -0.5 * np.pi)
        self.__broadcast_motion_command(command)

    # Move button callbacks
    def _move4(self):
        command = Motion("MoveBy", "DeadReckoning", 1, 10, 4)
        self.__broadcast_motion_command(command)
    def _move_4(self):
        command = Motion("MoveBy", "DeadReckoning", -1, 10, 4)
        self.__broadcast_motion_command(command)

    # Localize button callbacks
    def _localize_prox(self):
        self.broadcast(Signal("LocalizeProx", self.get_name(), self._robots[0].get_name(),
                              (None, None)))
    def _localize_psd(self):
        self.broadcast(Signal("LocalizePSD", self.get_name(), self._robots[0].get_name(),
                              (None, None)))

def main():
    """Runs test."""
    gui = GUILocalize()
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
