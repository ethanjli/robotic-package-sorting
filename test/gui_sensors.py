"""Script to test basic message-passing with GUIReactor.
Continuously reads out sensor values monitored by Monitor."""
import sys
import time
import Tkinter as tk

from components.messaging import Signal, Broadcaster
from components.concurrency import GUIReactor
from components.sensors import Monitor
from components.actions import Beeper
from robots.comm_usb import RobotComm

class GUISensors(GUIReactor, Broadcaster):
    """Shows a simple window with a hello world message and a quit button."""
    def __init__(self):
        super(GUISensors, self).__init__()
        self._comm = RobotComm(1, -50)
        self._robot = None
        self._threads = {}
        self._initialize_widgets()
    def _initialize_widgets(self):
        tk.Label(self._root, name="floorLabel", text="Floor: (?, ?)").pack()
        tk.Label(self._root, name="proximityLabel", text="Prox: (?, ?)").pack()
        tk.Label(self._root, name="psdLabel", text="PSD: ?").pack()
        tk.Button(self._root, name="connectButton",
                  text="Connect", command=self._connect).pack()
        tk.Button(self._root, name="monitorButton",
                  text="Monitor", command=self._toggle_monitor, state=tk.DISABLED).pack()
        tk.Button(self._root, name="beepButton",
                  text="Beep", command=self._beep, state=tk.DISABLED).pack()
        tk.Button(self._root, name="quitButton",
                  text="Quit", command=self.quit).pack()

    # Implementing abstract methods
    def _run_post(self):
        for _, thread in self._threads.items():
            thread.quit()
        for robot in self._comm.robotList:
            robot.reset()
        time.sleep(1.0)
        self._comm.stop()
    def _react(self, signal):
        if signal.Name == "Floor":
            self._root.nametowidget("floorLabel").config(text="Floor: ({}, {})"
                                                         .format(*signal.Data))
        elif signal.Name == "Proximity":
            self._root.nametowidget("proximityLabel").config(text="Prox: ({}, {})"
                                                             .format(*signal.Data))
        elif signal.Name == "PSD":
            self._root.nametowidget("psdLabel").config(text="PSD: {}"
                                                       .format(signal.Data))

    # Connect button callback
    def _connect(self):
        if not self._comm.start():
            raise RuntimeError("Robot communication failed to start.")
        self._root.nametowidget("connectButton").config(state=tk.DISABLED)
        self._robot = self._comm.robotList[0]
        self._initialize_threads()
        self._start_threads()
    def _initialize_threads(self):
        sensor_monitor = Monitor("Sensors Monitor", self._robot)
        self._threads["Sensors Monitor"] = sensor_monitor

        beeper = Beeper("Beeper", self._robot)
        self.register("Beep", beeper)
        self._threads["Beeper"] = beeper
    def _start_threads(self):
        for _, thread in self._threads.items():
            thread.start()
    def _activate_buttons(self):
        self._root.nametowidget("connectButton").config(text="Connected")
        self._root.nametowidget("monitorButton").config(state=tk.NORMAL)
        self._root.nametowidget("beepButton").config(state=tk.NORMAL)

    # Monitor button callback
    def _toggle_monitor(self):
        sensor_monitor = self._threads["Sensors Monitor"]
        sensor_monitor.toggle_registered("Floor", self)
        sensor_monitor.toggle_registered("Proximity", self)
        sensor_monitor.toggle_registered("PSD", self)
        monitor_button = self._root.nametowidget("monitorButton")
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
