"""Script to sort boxes."""
import sys
import Tkinter as tk
import tkMessageBox
import ttk

import numpy as np

from components.messaging import Signal
from components.geometry import Pose, to_vector
from components.util import clip
from components.robots import VirtualRobot, centroid_to_instant_center
from components.sensors import FilteringMonitor, VirtualMonitor
from components.world import Border, Wall, Package
from components.control import Motion, Localize, Pause, Wait, Finished
from components.control import PrimitiveController, SimplePrimitivePlanner
from components.app import Simulator

class DistributePlanner(SimplePrimitivePlanner):
    """Plans distribution of even-numbered and odd-numbered boxes."""
    def _generate_commands(self):
        commands = [
            Pause("Pause", 1),
            # Move to position to push box up
            Wait("Wait"),
            Motion("RotateTowards", "DeadReckoning", 1, 20, (0, -6)),
            Motion("MoveTo", "DeadReckoning", 1, 20, (0, None)),
            Motion("RotateTowards", "DeadReckoning", 1, 20, (0, 10)),
            # Push box up
            Motion("MoveTo", "DeadReckoning", 1, 20, (None, 0)),
            Finished("Finished", "Robot 1"),
            Motion("MoveTo", "DeadReckoning", 1, 20, (None, 8.5)),
            # Move to home
            Motion("MoveTo", "DeadReckoning", -1, 20, (None, 6)),
            Motion("RotateTowards", "DeadReckoning", -1, 20, (-6.5, 6)),
            Motion("MoveTo", "DeadReckoning", -1, 20, (-6.5, None)),
            # Move to position to push box down
            Wait("Wait"),
            Motion("RotateTowards", "DeadReckoning", 1, 20, (0, 6)),
            Motion("MoveTo", "DeadReckoning", 1, 20, (0, None)),
            Motion("RotateTowards", "DeadReckoning", 1, 20, (0, -10)),
            # Push box down
            Motion("MoveTo", "DeadReckoning", 1, 20, (None, 0)),
            Finished("Finished", "Robot 1"),
            Motion("MoveTo", "DeadReckoning", 1, 20, (None, -8.5)),
            # Move to home
            Motion("MoveTo", "DeadReckoning", -1, 20, (None, -6)),
            Motion("RotateTowards", "DeadReckoning", -1, 20, (-6.5, -6)),
            Motion("MoveTo", "DeadReckoning", -1, 20, (-6.5, None)),
            None
        ]
        while True:
            index = 0
            resetting = False
            while not resetting:
                resetting = not (yield commands[index])
                index = clip(0, len(commands) - 1, index + 1)
class DeliverPlanner(SimplePrimitivePlanner):
    """Plans delivery of boxes to the distributor robot."""
    def _generate_commands(self):
        commands = [
            Pause("Pause", 1),
            # Push box 1
            Motion("MoveTo", "DeadReckoning", 1, 20, (5, None)),
            Finished("Finished", "Robot 0"),
            Motion("MoveTo", "DeadReckoning", -1, 20, (24.5, None)),
            Finished("Finished", "GUISort"),
            # Push box 2
            Wait("Wait"),
            Wait("Wait"),
            Motion("MoveTo", "DeadReckoning", 1, 20, (5, None)),
            Finished("Finished", "Robot 0"),
            Motion("MoveTo", "DeadReckoning", -1, 20, (24.5, None)),
            None
        ]
        while True:
            index = 0
            resetting = False
            while not resetting:
                resetting = not (yield commands[index])
                index = clip(0, len(commands) - 1, index + 1)

class GUISort(Simulator):
    """Sorts boxes using two robots."""
    def __init__(self, name="Localization GUI", update_interval=10):
        super(GUISort, self).__init__(name, update_interval, 2)
        self._initialize_widgets()
        self._initialize_world(2)

    # Implementing parent abstract methods
    def _react_simulator(self, signal):
        if signal.Name == "Continue":
            print(self.get_name(), signal.Data)
        if signal.Name == "Moved":
            self.__set_motion_buttons_state("normal")
            self.__pauseresume_button.config(state="disabled", text="Pause")
        elif signal.Name == "Motion":
            self.__set_motion_buttons_state("disabled")
        elif signal.Name == "Continue" and signal.Data == self.get_name():
            self._ask_another_box()
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
        self.__move65_button = ttk.Button(move_frame, name="move65",
                                          text="Move 6.5", command=self._move65,
                                          state="disabled")
        self.__move65_button.pack(side="top", fill="x")
        self.__move_65_button = ttk.Button(move_frame, name="move-65",
                                           text="Move -6.5", command=self._move_65,
                                           state="disabled")
        self.__move_65_button.pack(side="top", fill="x")

        simulator_frame = ttk.LabelFrame(self._root, name="simulatorFrame",
                                         borderwidth=2, relief="ridge",
                                         text="Simulator")
        simulator_frame.pack(fill="both", expand="yes")
        self._initialize_simulator_widgets(simulator_frame, [-20, -20, 40, 20], 10)
    def _initialize_threads(self):
        self._add_virtual_world_threads()

        if self._robots[0].is_real():
            monitor_0 = FilteringMonitor("Monitor 0", self._robots[0])
        else:
            monitor_0 = VirtualMonitor("Monitor 0", self._robots[0], self._world)
        monitor_0.register("Floor", self._world)
        monitor_0.register("Proximity", self._world)
        monitor_0.register("PSD", self._world)
        self._add_thread(monitor_0)
        if self._robots[1].is_real():
            monitor_1 = FilteringMonitor("Monitor 1", self._robots[1])
        else:
            monitor_1 = VirtualMonitor("Monitor 1", self._robots[1], self._world)
        monitor_1.register("Floor", self._world)
        monitor_1.register("Proximity", self._world)
        monitor_1.register("PSD", self._world)
        self._add_thread(monitor_1)

        controller_0 = PrimitiveController("MotionController 0", self._robots[0], monitor_0)
        controller_0.register("Moved", self)
        self.register("Motion", controller_0)
        self.register("Stop", controller_0)
        self.register("Pause", controller_0)
        self.register("Resume", controller_0)
        controller_0.register("LocalizeProx", self._world)
        controller_0.register("LocalizePSD", self._world)
        self._add_thread(controller_0)

        planner_0 = DistributePlanner("DistributePlanner", self._robots[0])
        planner_0.register("Motion", controller_0)
        planner_0.register("Motion", self)
        planner_0.register("Localize", controller_0)
        planner_0.register("Stop", controller_0)
        controller_0.register("Moved", planner_0)
        self.register("Start", planner_0)
        self.register("Reset", planner_0)
        self._add_thread(planner_0)

        controller_1 = PrimitiveController("MotionController 1", self._robots[1], monitor_1)
        controller_1.register("Moved", self)
        self.register("Motion", controller_1)
        self.register("Stop", controller_1)
        self.register("Pause", controller_1)
        self.register("Resume", controller_1)
        controller_1.register("LocalizeProx", self._world)
        controller_1.register("LocalizePSD", self._world)
        self._add_thread(controller_1)

        planner_1 = DeliverPlanner("DeliverPlanner", self._robots[1])
        planner_1.register("Motion", controller_1)
        planner_1.register("Motion", self)
        planner_1.register("Localize", controller_1)
        planner_1.register("Stop", controller_1)
        controller_1.register("Moved", planner_1)
        self.register("Start", planner_1)
        self.register("Reset", planner_1)
        self._add_thread(planner_1)

        planner_0.register("Continue", planner_1)
        planner_1.register("Continue", planner_0)
        planner_1.register("Continue", self)
        self.register("Continue", planner_1)
    def _connect_post(self):
        self._change_reset_button("Reset")
        self._enable_start_button()
        self.__set_motion_buttons_state("normal")

        # Calibration
        self._robots[0].move_multiplier = 0.105
        self._robots[0].rotate_multiplier = 0.06
        self._robots[0].set_wheel_balance(11)
        self._robots[1].move_multiplier = 0.105
        self._robots[1].rotate_multiplier = 0.06
    def __set_motion_buttons_state(self, new_state):
        self.__rotate90_button.config(state=new_state)
        self.__rotate_90_button.config(state=new_state)
        self.__move65_button.config(state=new_state)
        self.__move_65_button.config(state=new_state)
    def _generate_virtual_robots(self):
        yield VirtualRobot("Robot 0",
                           pose=centroid_to_instant_center(Pose(to_vector(-6, -6), 0)),
                           servo_angle=(0.5 * np.pi))
        yield VirtualRobot("Robot 1",
                           pose=centroid_to_instant_center(Pose(to_vector(24, 0), np.pi)),
                           servo_angle=(0.5 * np.pi))
    def _populate_world(self):
        self._world.add_wall(Wall(0, center_x=-12, x_length=4, y_length=20))
        self._world.add_wall(Wall(1, center_x=-8, center_y=-12, x_length=4, y_length=4))
        self._world.add_wall(Wall(2, center_x=8, center_y=-12, x_length=4, y_length=4))
        self._world.add_wall(Wall(3, center_x=12, center_y=-11, x_length=4, y_length=10))
        self._world.add_wall(Wall(4, center_x=8, center_y=12, x_length=4, y_length=4))
        self._world.add_wall(Wall(5, center_x=12, center_y=11, x_length=4, y_length=10))
        self._world.add_wall(Wall(6, center_x=-8, center_y=12, x_length=4, y_length=4))
        self._world.add_wall(Wall(7, center_y=20, x_length=10, y_length=4))
    def _start_simulator(self):
        self.__stop_button.config(state="disabled")
        self.__set_motion_buttons_state("disabled")
        self.broadcast(Signal("Start", self.get_name(), self._robots[0].get_name(), None))
        self.broadcast(Signal("Start", self.get_name(), self._robots[1].get_name(), None))
    def _pause_simulator(self):
        self.broadcast(Signal("Pause", self.get_name(), self._robots[0].get_name(), None))
        self.broadcast(Signal("Pause", self.get_name(), self._robots[1].get_name(), None))
        self.__set_motion_buttons_state("normal")
    def _resume_simulator(self):
        self.broadcast(Signal("Resume", self.get_name(), self._robots[0].get_name(), None))
        self.broadcast(Signal("Resume", self.get_name(), self._robots[1].get_name(), None))
        self.__set_motion_buttons_state("disabled")
    def _reset_simulator_post(self):
        self.broadcast(Signal("Reset", self.get_name(), self._robots[0].get_name(), None))
        self.broadcast(Signal("Reset", self.get_name(), self._robots[1].get_name(), None))
        self.__set_motion_buttons_state("normal")

    def __broadcast_motion_command(self, command):
        self.broadcast(Signal("Motion", self.get_name(), self._robots[0].get_name(), command))
        self.broadcast(Signal("Motion", self.get_name(), self._robots[1].get_name(), command))
        self.__set_motion_buttons_state("disabled")
        self.__pauseresume_button.config(state="normal", text="Pause")
        self.__stop_button.config(state="normal")

    # Stop button callbacks
    def _stop(self):
        self.broadcast(Signal("Stop", self.get_name(), self._robots[0].get_name(), None))
        self.broadcast(Signal("Stop", self.get_name(), self._robots[1].get_name(), None))
        self.__set_motion_buttons_state("normal")
        self.__stop_button.config(state="disabled")
    def _pauseresume(self):
        state = self.__pauseresume_button.cget("text")
        if state == "Pause":
            self.broadcast(Signal("Pause", self.get_name(), self._robots[0].get_name(), None))
            self.broadcast(Signal("Pause", self.get_name(), self._robots[1].get_name(), None))
            self.__pauseresume_button.config(text="Resume")
        elif state == "Resume":
            self.broadcast(Signal("Resume", self.get_name(), self._robots[0].get_name(), None))
            self.broadcast(Signal("Resume", self.get_name(), self._robots[1].get_name(), None))
            self.__pauseresume_button.config(text="Pause")

    # Rotate button callbacks
    def _rotate90(self):
        command = Motion("RotateBy", "DeadReckoning", None, 20, 0.5 * np.pi)
        self.__broadcast_motion_command(command)
    def _rotate_90(self):
        command = Motion("RotateBy", "DeadReckoning", None, 20, -0.5 * np.pi)
        self.__broadcast_motion_command(command)

    # Move button callbacks
    def _move65(self):
        command = Motion("MoveBy", "DeadReckoning", 1, 20, 6.5)
        self.__broadcast_motion_command(command)
    def _move_65(self):
        command = Motion("MoveBy", "DeadReckoning", -1, 20, 6.5)
        self.__broadcast_motion_command(command)

    # Dialogue boxes
    def _ask_another_box(self):
        if tkMessageBox.askokcancel("Package Sorter", "Please add the next box."):
            self.broadcast(Signal("Continue", self.get_name(), self._robots[1].get_name(), "Robot 1"))

def main():
    """Runs test."""
    gui = GUISort("GUISort")
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
