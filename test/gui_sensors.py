"""Script to test basic message-passing with GUIReactor.
Continuously reads out sensor values monitored by Monitor."""
import sys
import Tkinter as tk

from components.messaging import Signal
from components.robots import RobotApp
from components.sensors import Monitor
from components.actions import Beeper

class GUISensors(RobotApp):
    """Reads out robot sensor values."""
    def __init__(self):
        super(GUISensors, self).__init__()
        self.__sensors_frame = None
        self.__effectors_frame = None
        self._initialize_widgets()

    # Implementing abstract methods
    def _react(self, signal):
        if signal.Name == "Floor":
            self.__sensors_frame.nametowidget("floorLabel").config(text="Floor: ({}, {})"
                                                                  .format(*signal.Data))
        elif signal.Name == "Proximity":
            self.__sensors_frame.nametowidget("proximityLabel").config(text="Prox: ({}, {})"
                                                                      .format(*signal.Data))
        elif signal.Name == "PSD":
            self.__sensors_frame.nametowidget("psdLabel").config(text="PSD: {}"
                                                                .format(signal.Data))
    def _initialize_widgets(self):
        app_frame = tk.Frame(self._root, name="appFrame",
                             borderwidth=2, relief=tk.RIDGE)
        app_frame.pack(fill=tk.X)
        self._initialize_robotapp_widgets(app_frame)

        self.__sensors_frame = tk.Frame(self._root, name="sensorsFrame",
                                       borderwidth=2, relief=tk.RIDGE)
        self.__sensors_frame.pack(fill=tk.X)
        tk.Label(self.__sensors_frame, name="floorLabel",
                 text="Floor: (?, ?)").pack(fill=tk.X)
        tk.Label(self.__sensors_frame, name="proximityLabel",
                 text="Prox: (?, ?)").pack(fill=tk.X)
        tk.Label(self.__sensors_frame, name="psdLabel",
                 text="PSD: ?").pack(fill=tk.X)
        tk.Button(self.__sensors_frame, name="monitorButton", text="Monitor",
                  command=self._toggle_monitor, state=tk.DISABLED).pack(fill=tk.X)

        self.__effectors_frame = tk.Frame(self._root, name="effectorsFrame",
                                         borderwidth=2, relief=tk.RIDGE)
        self.__effectors_frame.pack(fill=tk.X)
        tk.Button(self.__effectors_frame, name="beepButton",
                  text="Beep", command=self._beep, state=tk.DISABLED).pack(fill=tk.X)
    def _initialize_threads(self):
        sensor_monitor = Monitor("Sensors Monitor", self._robots[0])
        self._threads["Sensors Monitor"] = sensor_monitor

        beeper = Beeper("Beeper", self._robots[0])
        self.register("Beep", beeper)
        self._threads["Beeper"] = beeper
    def _connect_post(self):
        self.__sensors_frame.nametowidget("monitorButton").config(state=tk.NORMAL)
        self.__effectors_frame.nametowidget("beepButton").config(state=tk.NORMAL)

    # Monitor button callback
    def _toggle_monitor(self):
        sensor_monitor = self._threads["Sensors Monitor"]
        sensor_monitor.toggle_registered("Floor", self)
        sensor_monitor.toggle_registered("Proximity", self)
        sensor_monitor.toggle_registered("PSD", self)
        monitor_button = self.__sensors_frame.nametowidget("monitorButton")
        if sensor_monitor.is_registered("Floor", self):
            monitor_button.config(text="Stop Monitoring")
        else:
            monitor_button.config(text="Monitor")

    # Beep button callback
    def _beep(self):
        self.broadcast(Signal("Beep", (40, 0.2)))
        self.broadcast(Signal("Beep", (0, 0.1)))

def main():
    """Runs test."""
    gui = GUISensors()
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
