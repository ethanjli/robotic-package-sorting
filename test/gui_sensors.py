"""Script to test basic message-passing with GUIReactor.
Continuously reads out sensor values monitored by SimpleMonitor and FilteringMonitor."""
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
    def _react(self, signal):
        if signal.Name == "Floor":
            if signal.Sender == "Sensors Monitor":
                name = "floor"
                label = "Raw Floor"
            elif signal.Sender == "Filtering Sensors Monitor":
                label = "Filtered Floor"
                name = "filteredFloor"
            self.__sensors_frame.nametowidget(name).config(text="{}: ({:.0f}, {:.0f})"
                                                           .format(label, *signal.Data))
        elif signal.Name == "Proximity":
            if signal.Sender == "Sensors Monitor":
                name = "proximity"
                label = "Raw Prox"
            elif signal.Sender == "Filtering Sensors Monitor":
                name = "filteredProximity"
                label = "Filtered Prox"
            self.__sensors_frame.nametowidget(name).config(text="{}: ({:.0f}, {:.0f})"
                                                           .format(label, *signal.Data))
        elif signal.Name == "PSD":
            if signal.Sender == "Sensors Monitor":
                name = "psd"
                label = "Raw PSD"
            elif signal.Sender == "Filtering Sensors Monitor":
                name = "filteredPSD"
                label = "Filtered PSD"
            self.__sensors_frame.nametowidget(name).config(text="{}: {:.0f}"
                                                           .format(label, signal.Data))
    def _initialize_widgets(self):
        app_frame = ttk.LabelFrame(self._root, name="appFrame",
                                   borderwidth=2, relief="ridge",
                                   text="App")
        app_frame.pack(fill="x")
        self._initialize_robotapp_widgets(app_frame)

        self.__sensors_frame = ttk.LabelFrame(self._root, name="sensorsFrame",
                                              borderwidth=2, relief="ridge",
                                              text="Sensors")
        self.__sensors_frame.pack(fill="x")
        ttk.Label(self.__sensors_frame, name="floor",
                  text="Raw Floor: (?, ?)").pack(fill="x")
        ttk.Label(self.__sensors_frame, name="proximity",
                  text="Raw Prox: (?, ?)").pack(fill="x")
        ttk.Label(self.__sensors_frame, name="psd",
                  text="Raw PSD: ?").pack(fill="x")
        ttk.Label(self.__sensors_frame, name="filteredFloor",
                  text="Filtered Floor: (?, ?)").pack(fill="x")
        ttk.Label(self.__sensors_frame, name="filteredProximity",
                  text="Filtered Prox: (?, ?)").pack(fill="x")
        ttk.Label(self.__sensors_frame, name="filteredPSD",
                  text="Filtered PSD: ?").pack(fill="x")
        ttk.Button(self.__sensors_frame, name="monitor", text="Monitor",
                   command=self._toggle_monitor, state="disabled").pack(fill="x")

        self.__effectors_frame = ttk.LabelFrame(self._root, name="effectorsFrame",
                                                borderwidth=2, relief="ridge",
                                                text="Effectors")
        self.__effectors_frame.pack(fill="x")
        ttk.Button(self.__effectors_frame, name="beep1", text="Beep 1",
                   command=self._beep1, state="disabled").pack(fill="x")
        ttk.Button(self.__effectors_frame, name="beep2", text="Beep 2",
                   command=self._beep2, state="disabled").pack(fill="x")
        self.__servo_angle = tk.StringVar()
        self.__servo_angle.set("90")
        self.__servo_angle.trace("w", self._servo)
        angles = ttk.Combobox(self.__effectors_frame, name="servo",
                              textvariable=self.__servo_angle, state="disabled",
                              values=[str(i) for i in range(1, 181)])
        angles.pack(fill="x")
    def _initialize_threads(self):
        simple_monitor = SimpleMonitor("Sensors Monitor", self._robots[0])
        self.register("Servo", simple_monitor)
        self._threads["Sensors Monitor"] = simple_monitor

        filtered_monitor = FilteringMonitor("Filtering Sensors Monitor", self._robots[0])
        self.register("Servo", filtered_monitor)
        self._threads["Filtering Sensors Monitor"] = filtered_monitor

        beeper1 = Beeper("Beeper1", self._robots[0])
        self.register("Beep1", beeper1)
        self._threads["Beeper1"] = beeper1
        beeper2 = Beeper("Beeper2", self._robots[1])
        self.register("Beep2", beeper2)
        self._threads["Beeper2"] = beeper2
    def _connect_post(self):
        self.__sensors_frame.nametowidget("monitor").config(state="normal")
        self.__effectors_frame.nametowidget("beep1").config(state="normal")
        self.__effectors_frame.nametowidget("beep2").config(state="normal")
        self.__effectors_frame.nametowidget("servo").config(state="readonly")

    # Monitor button callback
    def _toggle_monitor(self):
        simple_monitor = self._threads["Sensors Monitor"]
        simple_monitor.toggle_registered("Floor", self)
        simple_monitor.toggle_registered("Proximity", self)
        simple_monitor.toggle_registered("PSD", self)

        filtered_monitor = self._threads["Filtering Sensors Monitor"]
        filtered_monitor.toggle_registered("Floor", self)
        filtered_monitor.toggle_registered("Proximity", self)
        filtered_monitor.toggle_registered("PSD", self)

        monitor_button = self.__sensors_frame.nametowidget("monitor")
        if simple_monitor.is_registered("Floor", self):
            monitor_button.config(text="Stop Monitoring")
        else:
            monitor_button.config(text="Monitor")

    # Beep button callback
    def _beep1(self):
        self.broadcast(Signal("Beep1", self.get_name(), (40, 0.2)))
        self.broadcast(Signal("Beep1", self.get_name(), (0, 0.1)))
    def _beep2(self):
        self.broadcast(Signal("Beep2", self.get_name(), (40, 0.2)))
        self.broadcast(Signal("Beep2", self.get_name(), (0, 0.1)))

    # Servo scale callback
    def _servo(self, _, dummy, operation):
        if operation == "w":
            self.broadcast(Signal("Servo", self.get_name(), int(self.__servo_angle.get())))

def main():
    """Runs test."""
    gui = GUISensors()
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
