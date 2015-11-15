"""Support for continuous monitoring of sensor data."""
import time
from threading import Semaphore

from messaging import Signal, Receiver, Broadcaster
from concurrency import InterruptableThread

_PSD_PORT = 0
_SERVO_PORT = 1
_PSD_STABILIZATION_INTERVAL = 0.8

class Monitor(InterruptableThread, Receiver, Broadcaster):
    """Monitors sensor values and broadcasts them to registered Reactors.
    Sleeps when no Reactors are currently registered.

    Signals Sent:
        Floor: Data is a 2-tuple of the left and right floor values.
        Proximity: Data is a 2-tuple of the left and right proximity values.
        PSD: Data is a positive int of the PSD scanner value. Only sends when the
        scanner has been given time to stabilize after servo motion.

    Signals Received:
        Received Signals are only processed when at least one Reactor is currently
        registered to receive Signals from the Monitor.
        Ignores None signals.
        Servo: rotates the PSD scanner. Data should be a positive int of the target angle.
    """
    def __init__(self, name, robot, warmup_time=1.0, update_interval=0.1):
        super(Monitor, self).__init__(name)
        self._robot = robot
        self._warmup_time = warmup_time
        self._update_interval = update_interval
        self._num_listeners = Semaphore(0) # tracks the number of registered Reactors
        self._psd_start_time = 0

    # Extending parent functions in Broadcaster
    def register(self, signal_name, reactor):
        """Registers a Reactor to listen for all signals of the specified name."""
        super(Monitor, self).register(signal_name, reactor)
        self._num_listeners.release()
    def deregister(self, signal_name, reactor):
        """Removes a Reactor that previously listened for signals."""
        super(Monitor, self).deregister(signal_name, reactor)
        self._num_listeners.acquire()

    # Implementation of parent abstract methods
    def _wake(self):
        self._num_listeners.release()
    def _run(self):
        self._run_pre()
        while not self._quit_flag.is_set():
            self._num_listeners.acquire() # sleep until at least one Reactor is registered
            self._num_listeners.release() # but preserve the number of registered Reactors
            self._react_all()
            self.broadcast(Signal("Floor", (self._robot.get_floor(0),
                                            self._robot.get_floor(1))))
            self.broadcast(Signal("Proximity", (self._robot.get_proximity(0),
                                                self._robot.get_proximity(1))))
            if time.time() >= self._psd_start_time:
                self.broadcast(Signal("PSD", self._robot.get_port(_PSD_PORT)))
            time.sleep(self._update_interval)
        self._run_post()
    def _react(self, signal):
        if signal.Name == "Servo":
            self._robot.set_port(_SERVO_PORT, signal.Data)
            self._psd_start_time = time.time() + _PSD_STABILIZATION_INTERVAL
    def _run_pre(self):
        time.sleep(self._warmup_time)
        self._robot.set_io_mode(_PSD_PORT, 0x0)
        self._robot.set_io_mode(_SERVO_PORT, 0x08)
        self._robot.set_port(_SERVO_PORT, 90)
        time.sleep(_PSD_STABILIZATION_INTERVAL)
    def _run_post(self):
        self._robot.set_port(_SERVO_PORT, 90)
        time.sleep(_PSD_STABILIZATION_INTERVAL)
