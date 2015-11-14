"""Simple Reactors that act on received Signals without sending Signals.
These classes usually send commands to a robot specified at instantiation."""
from reactors import Reactor
import time

class Echoer(Reactor):
    """Echoes any received Signals to stdout. Useful for debugging."""
    def __init__(self, name):
        super(Echoer, self).__init__(name, [])

    def _react(self, signal):
        print(signal)

class Beeper(Reactor):
    """Beeps. Useful for notifications.

    Signals:
        Will react to any Signal it receives, regardless of name. Data should be a
        2-tuple of the note and its duration.
    """
    def __init__(self, name, robot):
        super(Beeper, self).__init__(name, [])
        self._robot = robot

    def _react(self, signal):
        self._robot.set_musical_note(signal.Data[0])
        time.sleep(signal.Data[1])
    def _run_post(self):
        self._robot.set_musical_note(0)

class Mover(Reactor):
    """Moves the robot using its wheels.

    Signals:
        Advance: Data should be a positive int of the speed.
        Reverse: Data should be a positive int of the speed.
        Stop: Data is ignored.
        Rotate Left: Data should be a positive int of the speed.
        Rotate Right: Data should be a positive int of the speed.
    """
    def __init__(self, name, robot):
        super(Mover, self).__init__(name, [])
        self._robot = robot

    def _react(self, signal):
        if signal.Name == "Stop":
            self._stop()
        elif signal.Name == "Advance":
            self._robot.set_wheel(0, signal.Data)
            self._robot.set_wheel(1, signal.Data)
        elif signal.Name == "Reverse":
            self._robot.set_wheel(0, -signal.Data)
            self._robot.set_wheel(1, -signal.Data)
        elif signal.Name == "Rotate Left":
            self._robot.set_wheel(0, -signal.Data)
            self._robot.set_wheel(1, signal.Data)
        elif signal.Name == "Rotate Right":
            self._robot.set_wheel(0, signal.Data)
            self._robot.set_wheel(1, -signal.Data)
    def _run_pre(self):
        self._stop()
    def _run_post(self):
        self._stop()

    def _stop(self):
        self._robot.set_wheel(0, 0)
        self._robot.set_wheel(1, 0)

class ScannerRotator(Reactor):
    """Moves the PSD scanner.

    Signals:
        Will react to any Signal it receives, regardless of name. Data should be a
        positive int of the angle.
    """
    def __init__(self, name, robot, io_port=1):
        super(ScannerRotator, self).__init__(name, [])
        self._robot = robot
        self._port = io_port

    def _react(self, signal):
        self._robot.set_port(self._port, signal.Data)
    def _run_pre(self):
        self._robot.set_io_mode(self._port, 0x08)
        self._robot.set_port(self._port, 90)
    def _run_post(self):
        self._robot.set_port(self._port, 90)
