"""Utility functions/classes."""
from collections import deque
from struct import pack

def ordinal(number):
    """Returns the string ordinal for the input number.
    Algorithm from Gareth's solution at:
    http://codegolf.stackexchange.com/questions/4707/outputting-ordinal-numbers-1st-2nd-3rd
    """
    k = number % 10
    return "{}{}".format(number,
                         "tsnrhtdd"[(number / 10 % 10 != 1) * (k < 4) * k::4])

def rgb_to_hex(red, green, blue):
    """Returns hex code for an RGB color where values are integers between 0 and 255."""
    return "#" + pack("BBB", red, green, blue).encode("hex")

def rescale(in_min, in_max, out_min, out_max, value):
    """Rescales value from the specified range to the specified other range.

    Arguments:
        in_min: min bound of the range of value.
        in_max: max bound of the range of value.
        out_min: min bound of the range of the output.
        out_max: max bound of the range of the output.
        value: a number to rescale from the input range to the output range.
    Return:
        (float) a number that is the same relative distance away from out_min and
        out_max as value was from in_min and in_max
    """
    return (value - in_min) * (out_max - out_min) / float(in_max - in_min) + out_min
def clip(min_bound, max_bound, value):
    """Limits the range value may take.

    Arguments:
        min_bound: the lowest allowed value of value.
        max_bound: the highest allowed value of value.
        value: a whose value should be limited to being between min_bound
        and max_bound.
    Return:
        if value < min_bound, min_bound; if value > max_bound, max_bound;
        otherwise, value.
    """
    return max(min_bound, min(value, max_bound))
def within(bound_one, bound_two, value):
    """Checks whether the value is between the two bounds, inclusive."""
    return ((bound_one >= value and value >= bound_two)
            or (bound_one <= value and value <= bound_two))

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

