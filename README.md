# robotic-package-sorting
CS 123 final project to sort and deliver boxes in a warehouse using small "Hamster" robots.
Developed by Ethan Li and Geetanjali Johary.

## Components
The contents of the `components` package provide classes for concurrent management, sensing, and simulation of robots; and setup and execution of graphical interfaces for them. Graphical interfaces are written using `Tkinter` with `ttk` widgets. Concurrency is managed by message-passing between different threads and implemented to work like Actors (as in the concurrency model).

## Unit Tests
Unit tests are in the `test/` directory. To run a unit test with filename `foobar.py`, run `python -m test.foobar` from the root directory of this project. Or, if your python 2 distribution uses another command (e.g. `python2`), run with that command (e.g. `python2 -m test.foobar`).

## Dependencies
This project requires the proprietary Hamster API by Kre8 Technology, Inc, which depends on python 2. Thus, this project is written for compatibility with python 2. Bytecode of the Hamster API is distributed under the `hamster/` directory and documented in files under the `doc/` directory; it has been confirmed to work with Python 2.7.10.
