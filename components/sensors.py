"""Support for continuous monitoring of sensor data."""
import time
from threading import Semaphore

from numpy import median

from util import moving_average
from messaging import Signal, Receiver, Broadcaster
from concurrency import InterruptableThread

_WARMUP_TIME = 0.5

_PSD_STABILIZATION_INTERVAL = 0.8

_FLOOR_FILTER_WINDOW = 4
_PROXIMITY_FILTER_WINDOW = 8
_PSD_FILTER_WINDOW = 4

class Monitor(InterruptableThread, Receiver, Broadcaster):
    """Abstract class for monitoring sensor values and broadcasting them.
    Can be set to sleep when no Reactors are currently registered.

    Signals Sent:
        Floor: Data is a 2-tuple of the left and right floor values.
        Proximity: Data is a 2-tuple of the left and right proximity values.
        PSD: Data is a positive int of the PSD scanner value. Only updates when the
        scanner has been given time to stabilize after servo motion.

    Signals Received:
        Received Signals are only processed when at least one Reactor is currently
        registered to receive Signals from the Monitor. Signals are processed
        regardless of sender. Ignores None signals.
        Servo: rotates the PSD scanner. Data should be a positive int of the target angle.
    """
    def __init__(self, name, robot, update_interval=0.1, auto_sleep=True):
        super(Monitor, self).__init__(name)
        self._robot = robot
        self._update_interval = update_interval
        self.__auto_sleep = auto_sleep
        self._num_listeners = Semaphore(0) # tracks the number of registered Reactors
        self._psd_start_time = 0

    # Extending parent functions in Broadcaster
    def register(self, signal_name, reactor):
        """Registers a Reactor to listen for all signals of the specified name."""
        super(Monitor, self).register(signal_name, reactor)
        if self.__auto_sleep:
            self._num_listeners.release()
    def deregister(self, signal_name, reactor):
        """Removes a Reactor that previously listened for signals."""
        super(Monitor, self).deregister(signal_name, reactor)
        if self.__auto_sleep:
            self._num_listeners.acquire()

    # Implementation of parent abstract methods
    def _wake(self):
        self._num_listeners.release()
    def _run(self):
        self._run_pre()
        while not self._quit_flag.is_set():
            if self.__auto_sleep:
                self._num_listeners.acquire()
                self._num_listeners.release()
            self._react_all()
            self._update_floor()
            self._update_proximity()
            if time.time() >= self._psd_start_time:
                self._update_psd()
            time.sleep(self._update_interval)
        self._run_post()
    def _react(self, signal):
        if signal.Name == "Servo":
            self._robot.servo(signal.Data)
            self._psd_start_time = time.time() + _PSD_STABILIZATION_INTERVAL
            self._react_servo_post()
    def _run_pre(self):
        time.sleep(_WARMUP_TIME)
        self._robot.init_psd_scanner()
        self._robot.servo(90)
        time.sleep(_PSD_STABILIZATION_INTERVAL)
    def _run_post(self):
        self._robot.servo(90)
        time.sleep(_PSD_STABILIZATION_INTERVAL)

    # Abstract methods
    def _update_floor(self):
        """Read and process floor sensor values and broadcast values."""
        pass
    def _update_proximity(self):
        """Read and process proximity sensor values and broadcast values."""
        pass
    def _update_psd(self):
        """Read and process PSD sensor value and broadcast value."""
        pass
    def _react_servo_post(self):
        """Executes after reacting to a Servo signal."""
        pass

class SimpleMonitor(Monitor):
    """A Monitor that updates sensor values.
    The reference implementation of Monitor.
    """
    def __init__(self, name, robot, update_interval=0.1):
        super(SimpleMonitor, self).__init__(name, robot, update_interval)

    # Implementation of parent abstract methods
    def _update_floor(self):
        floor = self._robot.get_floor()
        self.broadcast(Signal("Floor", self.get_name(), floor))
    def _update_proximity(self):
        proximity = self._robot.get_proximity()
        self.broadcast(Signal("Proximity", self.get_name(), proximity))
    def _update_psd(self):
        psd = self._robot.get_psd()
        self.broadcast(Signal("PSD", self.get_name(), psd))

class FilteringMonitor(Monitor):
    """A Monitor that updates low-pass filtered sensor values.
    Always collects and filters sensor data.
    """
    def __init__(self, name, robot, update_interval=0.1):
        super(FilteringMonitor, self).__init__(name, robot, update_interval, False)
        self._floor_left_filter = moving_average(_FLOOR_FILTER_WINDOW, median)
        self._floor_right_filter = moving_average(_FLOOR_FILTER_WINDOW, median)
        self._proximity_left_filter = moving_average(_PROXIMITY_FILTER_WINDOW, median)
        self._proximity_right_filter = moving_average(_PROXIMITY_FILTER_WINDOW, median)
        self._psd_filter = moving_average(_PSD_FILTER_WINDOW, median)

    # Implementation of parent abstract methods
    def _react_servo_post(self):
        self._psd_filter.send(None)
    def _update_floor(self):
        floor = self._robot.get_floor()
        floor_filtered = (self._floor_left_filter.send(floor[0]),
                          self._floor_right_filter.send(floor[1]))
        for value in floor_filtered:
            if value is None:
                return
        self.broadcast(Signal("Floor", self.get_name(), floor_filtered))
    def _update_proximity(self):
        proximity = self._robot.get_proximity()
        proximity_filtered = (self._proximity_left_filter.send(proximity[0]),
                              self._proximity_right_filter.send(proximity[1]))
        for value in proximity_filtered:
            if value is None:
                return
        self.broadcast(Signal("Proximity", self.get_name(), proximity_filtered))
    def _update_psd(self):
        psd = self._robot.get_psd()
        psd_filtered = self._psd_filter.send(psd)
        if psd_filtered is None:
            return
        self.broadcast(Signal("PSD", self.get_name(), psd_filtered))
