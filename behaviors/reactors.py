"""Convenience classes to support easier management of execution primitives
for concurrency."""
import threading
from collections import namedtuple
import Queue as queue

# Signals are the messages passed around by Reactors for inter-thread communication.
Signal = namedtuple("Signal", ["Name", "Data"])

class InterruptableThread(object):
    """Interface class to support easier threading."""
    def __init__(self, name):
        super(InterruptableThread, self).__init__()
        self._name = name
        self._quit_flag = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name)

    def get_name(self):
        """Returns the name of the thread instance as specified during instantiation."""
        return self._name

    # Threading
    def start(self):
        """Starts the thread."""
        self._thread.start()
    def quit(self):
        """Quits and joins with the thread, if it's running."""
        self._quit_flag.set()
        self._wake()
        if self._thread.is_alive():
            self._thread.join()

    # Abstract methods
    def _wake(self):
        """Wake the thread method, if it's sleeping, to signal it to prepare to quit."""
        pass
    def _run(self):
        """The function that will be run in a new thread."""
        pass

class Reactor(InterruptableThread):
    """Models an event-driven thread that receives & handles Signals as messages."""
    def __init__(self, name, QueueType=queue.Queue):
        super(Reactor, self).__init__(name)
        self._queue = QueueType()

    # Message-passing
    def send(self, signal):
        """Receives a Signal and wakes the thread if it is sleeping."""
        self._queue.put(signal)
    def clear(self):
        """Discards all waiting Signals. Useful if the Reactor was sleeping."""
        while not self._queue.empty():
            try:
                self._queue.get(False)
            except queue.Empty:
                continue
            self._queue.task_done()

    # Implementation of parent abstract methods
    def _wake(self):
        """Wakes the thread to prepare it to quit."""
        self._queue.put(None)
    def _run(self):
        """Runs as a thread."""
        self._run_pre()
        while not self._quit_flag.is_set():
            signal = self._queue.get()
            if signal is not None:
                self._react(signal)
            else:
                self._quit_flag.set()
            self._queue.task_done()
        self._run_post()

    # Abstract methods
    def _react(self, signal):
        """Processes a Signal."""
        pass
    def _run_pre(self):
        """Executes before the thread starts. Useful for initialization."""
        pass
    def _run_post(self):
        """Executes after the thread ends. Useful for initialization."""
        pass

class Signaler(object):
    """Provides mixin functionality to broadcast Signals to groups of Reactors.
    Reactors can be registered to listen to Signals (based on Signal name)
    that the Signaler emits.
    """
    def __init__(self):
        super(Signaler, self).__init__()
        self._reactors = {}

    def register_reactor(self, signal_name, reactor):
        """Registers a Reactor to listen for all signals of the specified name.

        Arguments:
            signal_name: the name of the type of Signal to listen to.
            reactor: a Reactor.
        """
        if signal_name not in self._reactors:
            self._reactors[signal_name] = {}
        self._reactors[signal_name].add(reactor)

    def deregister_reactor(self, signal_name, reactor):
        """Removes a Reactor that previously listened for signals.

        Arguments:
            signal_name: the name of the type of Signal to listen to.
            reactor: a Reactor that was previously registered to listen to signals
            of type signal_name.

        Exceptions:
            ValueError: no Reactor has ever been registered to listen to signals of
            type signal_name.
            ValueError: the provided Reactor is not currently registered to listen to
            signals of type signal_name.
        """
        if signal_name not in self._reactors:
            raise ValueError("No Reactor has ever been registered to listen to "
                             "\"{}\" signals".format(signal_name))
        if reactor not in self._reactors[signal_name]:
            raise ValueError("Reactor \"{}\" is not currently registered to listen "
                             "to \"{}\" signals".format(reactor.get_name(), signal_name))
        self._reactors[signal_name].remove(reactor)

    def broadcast(self, signal):
        """Broadcasts a signal to all Reactors registered with the specified Signal's name.
        If no Reactor is registered with the specified Signal's name, does nothing.

        Arguments:
            signal: the signal to broadcast.
        """
        if signal.Name not in self._reactors:
            return
        for reactor in self._reactors[signal.Name]:
            reactor.send(signal)

