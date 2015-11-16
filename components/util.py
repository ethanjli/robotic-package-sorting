"""Utility functions/classes."""
from collections import deque

def ordinal(number):
    """Returns the string ordinal for the input number.
    Algorithm from Gareth's solution at:
    http://codegolf.stackexchange.com/questions/4707/outputting-ordinal-numbers-1st-2nd-3rd
    """
    k = number % 10
    return "{}{}".format(number,
                         "tsnrhtdd"[(number / 10 % 10 != 1) * (k < 4) * k::4])

def initialized_coroutine(function):
    """Function decorator to automatically initialize a coroutine."""
    def _wrapper(*args, **kw):
        coroutine = function(*args, **kw)
        coroutine.send(None)
        return coroutine
    return _wrapper

def unweighted_mean(container):
    """Returns the unweighted mean of the container (must be iterable and have len)."""
    return sum(container) / float(len(container))

@initialized_coroutine
def moving_average(max_samples, averager=unweighted_mean):
    """A generator to compute the moving average of a signal.
    To initialize, assign to a variable and start using it - no need to call the next
    function on it or send in an initial None value.

    Arguments:
        max_samples: the window size over which to compute the moving average.

    Sending:
        Send values into moving_average to add them to the signal.
        Send None into mvoing_average to reset the signal.

    Yielding:
        None: if the filter has not yet collected max_samples samples.
        Otherwise, the average of the max_samples most recent samples.
    """
    signal = deque(maxlen=max_samples)
    average = None
    while True:
        value = yield average
        if value is None:
            signal = deque(maxlen=max_samples)
            average = None
        else:
            signal.append(value)
            num_samples = len(signal)
            if num_samples == max_samples:
                average = averager(signal)
            else:
                average = None

