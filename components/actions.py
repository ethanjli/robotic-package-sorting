"""Simple Reactors that act on received Signals without sending Signals.
These classes usually send commands to a robot specified at instantiation."""
import time

from components.concurrency import Reactor

SERVO_PORT = 1

class Echoer(Reactor):
    """Echoes any received Signals to stdout. Useful for debugging."""
    def __init__(self, name):
        super(Echoer, self).__init__(name)

    def _react(self, signal):
        print(signal)

class Beeper(Reactor):
    """Beeps. Useful for notifications.

    Signals Received:
        Will react to any Signal, regardless of name or sender. Data should be a
        2-tuple of the note and its duration.
    """
    def __init__(self, name, robot):
        super(Beeper, self).__init__(name)
        self._robot = robot

    def _react(self, signal):
        self._robot.beep(signal.Data[0])
        time.sleep(signal.Data[1])
    def _run_post(self):
        self._robot.beep(0)

class Mover(Reactor):
    """Moves the robot using its wheels.

    Signals Received:
        Will react to any Signal of correct name, regardless of sender.
        Advance: Data should be a positive int of the speed.
        Reverse: Data should be a positive int of the speed.
        Stop: Data is ignored.
        Rotate Left: Data should be a positive int of the speed.
        Rotate Right: Data should be a positive int of the speed.
    """
    def __init__(self, name, robot):
        super(Mover, self).__init__(name)
        self._robot = robot

    def _react(self, signal):
        if signal.Name == "Stop":
            self._stop()
        elif signal.Name == "Advance":
            self._robot.move(signal.Data)
        elif signal.Name == "Reverse":
            self._robot.move(-signal.Data)
        elif signal.Name == "Rotate Left":
            self._robot.rotate(signal.Data)
        elif signal.Name == "Rotate Right":
            self._robot.rotate(-signal.Data)
    def _run_pre(self):
        self._stop()
    def _run_post(self):
        self._stop()

    def _stop(self):
        self._robot.move(0)
