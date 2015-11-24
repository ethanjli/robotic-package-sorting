"""Convenience classes to support Actor model of concurrency, backed by threads."""
import threading
from collections import namedtuple
import Queue as queue

from components.messaging import Receiver

class InterruptableThread(object):
    """Interface class to support easier threading."""
    def __init__(self, name):
        super(InterruptableThread, self).__init__()
        self.__name = name
        self.__quit_flag = threading.Event()
        self.__thread = threading.Thread(target=self._run, name=name)

    def get_name(self):
        """Returns the name of the thread instance as specified during instantiation."""
        return self.__name

    # Threading
    def start(self):
        """Starts the thread."""
        self.__thread.start()
    def will_quit(self):
        """Checks whether the thread is supposed to quit."""
        return self.__quit_flag.is_set()
    def quit(self):
        """Quits and joins with the thread, if it's running."""
        self._quit_soon()
        self._wake()
        if self.__thread.is_alive():
            self.__thread.join()
    def _quit_soon(self):
        """Marks the flag that the thread will check to quit."""
        self.__quit_flag.set()

    # Abstract methods
    def _wake(self):
        """Wake the thread method, if it's sleeping, to signal it to prepare to quit."""
        pass
    def _run(self):
        """The function that will be run in a new thread."""
        pass

class Reactor(InterruptableThread, Receiver):
    """Models an event-driven thread that receives & handles Signals as messages."""
    def __init__(self, name):
        super(Reactor, self).__init__(name)

    # Implementation of parent abstract methods
    def _wake(self):
        """Wakes the thread to prepare it to quit."""
        self.send(None)
    def _run(self):
        """Runs as a thread."""
        self._run_pre()
        while not self.will_quit():
            signal = self._receive()
            if signal is not None:
                self._react(signal)
            else:
                self._quit_soon()
            self._received_done()
        self._run_post()

    # Abstract methods
    def _run_pre(self):
        """Executes before the thread starts. Useful for initialization."""
        pass
    def _run_post(self):
        """Executes after the thread ends. Useful for initialization."""
        pass
