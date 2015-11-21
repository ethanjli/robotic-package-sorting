"""Mixin classes to support queue-based message-passing between objects."""
from collections import namedtuple
import Queue as queue

# Signals are the messages passed around by Receivers for inter-thread communication.
# Name is the name of a Signal. Broadcasters can broadcast a Signal to all Receivers
# registered to listen to that name.
# Sender is the name of the object that sent the Signal, and usually corresponds to
# the name of an InterruptableThread.
# Namespace is the name of the namespace of the Signal's Name, and may correspond
# to the name or ID of a robot. Along with Sender, it helps Receivers decide which
# Signals to handle and how to do so.
# Data is the payload data, and may correspond to a single value, a tuple, etc.
Signal = namedtuple("Signal", ["Name", "Sender", "Namespace", "Data"])

class Receiver(object):
    """Provides mixin functionality to receive Signals."""
    def __init__(self):
        super(Receiver, self).__init__()
        self.__queue = queue.Queue()

    # Message-passing
    def send(self, signal):
        """Receives a Signal and wakes the thread if it is sleeping."""
        self.__queue.put(signal)
    def clear(self):
        """Discards all waiting Signals. Useful if the Receiver was sleeping."""
        while not self.__queue.empty():
            try:
                self.__queue.get(False)
            except queue.Empty:
                continue
            self.__queue.task_done()
    def _receive(self):
        """Gets the next Signal from the queue. Blocks until it can do so."""
        return self.__queue.get()
    def _received_done(self):
        """Acknowledges that the Signal received has been processed.
        Only useful for other threads that are joining the __queue.
        """

    # Message processing
    def _react_all(self):
        """Reacts to all received Signals.
        Blocks until all received Signals, if any exist, have been reacted to.
        Delegates any special handling of None signals to the _react method: to
        quit from within _react, raise the queue.Empty exception.
        """
        while not self.__queue.empty(): # Process all waiting Signals
            try:
                self._react(self.__queue.get(False))
            except queue.Empty:
                continue
            self._received_done()

    # Abstract methods
    def _react(self, signal):
        """Processes a Signal."""
        pass

class Broadcaster(object):
    """Provides mixin functionality to broadcast Signals to groups of Receivers.
    Receivers can be registered to listen to Signals (based on Signal name)
    that the Broadcaster emits.
    """
    def __init__(self):
        super(Broadcaster, self).__init__()
        self.__receivers = {}

    def register(self, signal_name, receiver):
        """Registers a Receiver to listen for all signals of the specified name.

        Arguments:
            signal_name: the name of the type of Signal to listen to.
            receiver: a Receiver.
        """
        if signal_name not in self.__receivers:
            self.__receivers[signal_name] = set()
        self.__receivers[signal_name].add(receiver)
    def deregister(self, signal_name, receiver):
        """Removes a Receiver that previously listened for signals.

        Arguments:
            signal_name: the name of the type of Signal to listen to.
            receiver: a Receiver that was previously registered to listen to signals
            of type signal_name.

        Exceptions:
            ValueError: no Receiver has ever been registered to listen to signals of
            type signal_name.
            ValueError: the provided Receiver is not currently registered to listen to
            signals of type signal_name.
        """
        if signal_name not in self.__receivers:
            raise ValueError("No Receiver has ever been registered to listen to "
                             "\"{}\" signals".format(signal_name))
        if receiver not in self.__receivers[signal_name]:
            raise ValueError("Receiver \"{}\" is not currently registered to listen "
                             "to \"{}\" signals".format(receiver.get_name(), signal_name))
        self.__receivers[signal_name].remove(receiver)
    def is_registered(self, signal_name, receiver):
        """Checks whether a Receiver is currently listening for signals."""
        return signal_name in self.__receivers and receiver in self.__receivers[signal_name]
    def toggle_registered(self, signal_name, receiver):
        """Toggles whether a Receiver is currently listening for signals."""
        if self.is_registered(signal_name, receiver):
            self.deregister(signal_name, receiver)
        else:
            self.register(signal_name, receiver)

    def broadcast(self, signal):
        """Broadcasts a signal to all Receiver registered with the specified Signal's name.
        If no Receiver is registered with the specified Signal's name, does nothing.

        Arguments:
            signal: the signal to broadcast.
        """
        if signal.Name not in self.__receivers:
            return
        for receiver in self.__receivers[signal.Name]:
            receiver.send(signal)

