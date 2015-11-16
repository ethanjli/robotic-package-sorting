"""Tests the moving_average generator in components.sensors with user input."""
import sys
from collections import deque
from components.sensors import moving_average

MAX_SIZE = 5

def main():
    """Runs test."""
    signal = deque(maxlen=MAX_SIZE)
    print("Running a moving average over {} samples.".format(MAX_SIZE))
    print("Enter a \"quit\" to exit or \"clear\" to reset the moving average.")
    print("Enter a floating-point number to continue the moving average.")
    averager = moving_average(MAX_SIZE)
    while True:
        next_val = raw_input("> ")
        if next_val == "quit":
            break
        elif next_val == "clear":
            signal = deque(maxlen=MAX_SIZE)
            averager.send(None)
        else:
            signal.append(float(next_val))
            print("Average of {} is {}".format(signal, averager.send(float(next_val))))

if __name__ == "__main__":
    sys.exit(main())
