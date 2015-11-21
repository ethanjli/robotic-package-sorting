"""Script to test connection & control of multiple robots."""
import sys
import Tkinter as tk
import ttk

from components.messaging import Signal
from components.robots import RobotApp, Beeper
from components.sensors import SimpleMonitor, FilteringMonitor

class GUISensors(RobotApp):
    """Reads out robot sensor values."""
    def __init__(self, name="Sensors GUI", update_interval=10):
        super(GUISensors, self).__init__(name, update_interval, 2)
        self.__sensors_frame = None
        self.__effectors_frame = None
        self._initialize_widgets()

    # Implementing abstract methods
    def _initialize_widgets(self):
        app_frame = ttk.LabelFrame(self._root, name="appFrame",
                                   borderwidth=2, relief="ridge",
                                   text="App")
        app_frame.pack(fill="x")
        self._initialize_robotapp_widgets(app_frame)

        self.__effectors_frame = ttk.LabelFrame(self._root, name="effectorsFrame",
                                                borderwidth=2, relief="ridge",
                                                text="Effectors")
        self.__effectors_frame.pack(fill="x")
        ttk.Button(self.__effectors_frame, name="beep1", text="Beep 1",
                   command=self._beep1, state="disabled").pack(fill="x")
        ttk.Button(self.__effectors_frame, name="beep2", text="Beep 2",
                   command=self._beep2, state="disabled").pack(fill="x")
        self.__servo_angle1 = tk.StringVar()
        self.__servo_angle1.set("90")
        self.__servo_angle1.trace("w", self._servo1)
        angles1 = ttk.Combobox(self.__effectors_frame, name="servo1",
                               textvariable=self.__servo_angle1, state="disabled",
                               values=[str(i) for i in range(1, 181)])
        angles1.pack(fill="x")
        self.__servo_angle2 = tk.StringVar()
        self.__servo_angle2.set("90")
        self.__servo_angle2.trace("w", self._servo2)
        angles2 = ttk.Combobox(self.__effectors_frame, name="servo2",
                               textvariable=self.__servo_angle2, state="disabled",
                               values=[str(i) for i in range(1, 181)])
        angles2.pack(fill="x")
    def _initialize_threads(self):
        monitor1 = SimpleMonitor("Monitor1", self._robots[0], 0.1, False)
        self.register("Servo", monitor1)
        self._threads["Monitor1"] = monitor1
        monitor2 = SimpleMonitor("Monitor2", self._robots[1], 0.1, False)
        self.register("Servo", monitor2)
        self._threads["Monitor2"] = monitor2

        beeper1 = Beeper("Beeper1", self._robots[0])
        self.register("Beep", beeper1)
        self._threads["Beeper1"] = beeper1
        beeper2 = Beeper("Beeper2", self._robots[1])
        self.register("Beep", beeper2)
        self._threads["Beeper2"] = beeper2
    def _connect_post(self):
        self.__effectors_frame.nametowidget("beep1").config(state="normal")
        self.__effectors_frame.nametowidget("beep2").config(state="normal")
        self.__effectors_frame.nametowidget("servo1").config(state="readonly")
        self.__effectors_frame.nametowidget("servo2").config(state="readonly")

    # Beep button callback
    def _beep1(self):
        self.broadcast(Signal("Beep", self.get_name(), repr(self._robots[0]), (40, 0.2)))
        self.broadcast(Signal("Beep", self.get_name(), repr(self._robots[0]), (0, 0.1)))
    def _beep2(self):
        self.broadcast(Signal("Beep", self.get_name(), repr(self._robots[1]), (40, 0.2)))
        self.broadcast(Signal("Beep", self.get_name(), repr(self._robots[1]), (0, 0.1)))

    # Servo scale callback
    def _servo1(self, _, dummy, operation):
        if operation == "w":
            self.broadcast(Signal("Servo", self.get_name(), repr(self._robots[0]),
                                  int(self.__servo_angle1.get())))
    def _servo2(self, _, dummy, operation):
        if operation == "w":
            self.broadcast(Signal("Servo", self.get_name(), repr(self._robots[1]),
                                  int(self.__servo_angle2.get())))

def main():
    """Runs test."""
    gui = GUISensors()
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
