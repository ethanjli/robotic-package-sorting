"""Support for continuous monitoring of sensor data."""
from reactors import Signal, InterruptableThread, Signaler
import time
from threading import Semaphore

# The types of Signals the Monitor can send to registered Reactors.
SIGNAL_NAMES = (
    "Floor", # Data is a 2-tuple of the left and right floor values.
    "Proximity", # Data is a 2-tuple of the left and right proximity values.
    "PSD", # Data is a positive int.
)

PSD_PORT = 0

class Monitor(InterruptableThread, Signaler):
    """Monitors sensor values and broadcasts them to registered Reactors.
    Sleeps when no Reactors are currently registered."""
    def __init__(self, name, robot, warmup_time=1.0, update_interval=0.1):
        super(Monitor, self).__init__(name)
        self._robot = robot
        self._warmup_time = warmup_time
        self._update_interval = update_interval
        self._num_listeners = Semaphore(0) # tracks the number of registered Reactors

    # Extending parent functions in Signaler
    def register_reactor(self, signal_name, reactor):
        """Registers a Reactor to listen for all signals of the specified name."""
        super(Monitor, self).register_reactor(signal_name, reactor)
        self._num_listeners.release()
    def deregister_reactor(self, signal_name, reactor):
        """Removes a Reactor that previously listened for signals."""
        super(Monitor, self).deregister_reactor(signal_name, reactor)
        self._num_listeners.acquire()

    def _wake(self):
        self._num_listeners.release()
    def _run_pre(self):
        time.sleep(self._warmup_time)
        self._robot.set_io_mode(PSD_PORT, 0x0)
    def _run(self):
        self._run_pre()
        while not self._quit_flag.is_set():
            self._num_listeners.acquire() # sleep until at least one Reactor is registered
            self._num_listeners.release() # but preserve the number of registered Reactors
            self.broadcast(Signal("Floor", (self._robot.get_floor(0),
                                            self._robot.get_floor(1))))
            self.broadcast(Signal("Proximity", (self._robot.get_proximity(0),
                                                self._robot.get_proximity(1))))
            self.broadcast(Signal("PSD", self._robot.get_port(PSD_PORT)))
            time.sleep(self._update_interval)

