"""Mixin classes to support queue-based message-passing between objects."""
from collections import namedtuple
import Queue as queue

# Signals are the messages passed around by Reactors for inter-thread communication.
Signal = namedtuple("Signal", ["Name", "Data"])

class Receiver(object):
    """Provides mixin functionality to receive Signals."""
    def __init__(self):
        super(Receiver, self).__init__()
        self._queue = queue.Queue()

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

class Broadcaster(object):
    """Provides mixin functionality to broadcast Signals to groups of Reactors.
    Reactors can be registered to listen to Signals (based on Signal name)
    that the Broadcaster emits.
    """
    def __init__(self):
        super(Broadcaster, self).__init__()
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

