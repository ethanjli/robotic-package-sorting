"""Classes to setup and run graphical programs involving robots."""
import time
import Tkinter as tk
import tkMessageBox
import ttk

from components.util import ordinal
from components.messaging import Receiver, Broadcaster
from components.robots import HamsterComm, Robot
from components.geometry import scale_bounds
from components.world import VirtualWorld

MIN_RSSI = -50

class AutoScrollbar(ttk.Scrollbar):
    """A scrollbar that automatically hides if unneeded."""
    def set(self, low, high):
        """The callback command to set the scrollbar."""
        orientation = str(self.cget("orient")).capitalize()
        if float(low) <= 0.0 and float(high) >= 1.0:
            self.config(style="Hidden.{}.TScrollbar".format(orientation))
        else:
            self.config(style="{}.TScrollbar".format(orientation))
        tk.Scrollbar.set(self, low, high)

class GUIReactor(Receiver):
    """Runs and updates a TKinter root window based on received Signals.
    Has the same public semantics as InterruptableThreads, but the start method blocks
    the caller until the GUI exits.
    Has the same abstract method semantics as Reactors. Unlike Reactors, this
    must run in the main thread.
    """
    def __init__(self, name="GUI", update_interval=10):
        super(GUIReactor, self).__init__()
        self.__name = name
        self._root = tk.Tk()
        self._root.title(name)
        self.__initialize_theming()
        self.__update_interval = update_interval
    def __initialize_theming(self):
        style = ttk.Style()
        # Set theme
        if "aqua" in style.theme_names():
            style.theme_use("aqua") # OS X
        elif "vista" in style.theme_names():
            style.theme_use("vista") # Windows
        else:
            style.theme_use("clam") # Linux
        # Add custom styles
        style.configure("Hidden.Horizontal.TScrollbar", arrowsize=0)
        style.configure("Hidden.Vertical.TScrollbar", arrowsize=0)

        self._root.style = style

    def get_name(self):
        """Returns the name of the thread instance as specified during instantiation."""
        return self.__name

    # Event loop
    def start(self):
        """Starts the GUI and blocks until the GUI quits.
        Because of how TKinter works, this should be called in the main thread."""
        self._run_pre()
        self._run()
        self._root.mainloop()
    def quit(self):
        """Quits the GUI and unblocks the thread that called the start method."""
        self._root.quit()
        self._run_post()
    def _run(self):
        """Reacts to any received signals and then sleeps for a bit."""
        self._react_all()
        self._root.after(self.__update_interval, self._run)

    # Abstract methods
    def _run_pre(self):
        """Executes before the GUI starts. Useful for initialization."""
        pass
    def _run_post(self):
        """Executes after the thread ends. Useful for initialization."""
        pass

class RobotApp(GUIReactor, Broadcaster):
    """Shows a simple window with a hello world message and a quit button."""
    def __init__(self, name="App", update_interval=10, num_robots=1):
        super(RobotApp, self).__init__(name, update_interval)
        self.__hamster_comm = None
        self._num_robots = num_robots
        self.__virtual_robot_generator = self._generate_virtual_robots()
        self._robots = []
        self._threads = {}
        self.__connect_button = None
        self.__quit_button = None

    # Implementing abstract methods
    def _run_post(self):
        for _, thread in self._threads.items():
            thread.quit()
        if self.__hamster_comm is None:
            return
        for hamster_robot in self.__hamster_comm.robotList:
            hamster_robot.reset()
        time.sleep(1.0)
        self.__hamster_comm.stop()

    # Utility for subclasses
    def _initialize_robotapp_widgets(self, parent):
        """Add RobotApp widgets into the specified parent widget.

        Widgets:
            connect: a button to connect to the robots.
            quit: a button to exit the application.
        """
        self.__connect_button = ttk.Button(parent, name="connect", text="Connect",
                                           command=self.__connect_all)
        self.__connect_button.pack(side="left", fill="y")
        self.__quit_button = ttk.Button(parent, name="quit", text="Quit",
                                        command=self.quit)
        self.__quit_button.pack(side="left", fill="y")

    # Connect button callback
    def __connect_all(self):
        self.__connect_button.config(state="disabled")
        while len(self._robots) < self._num_robots:
            if not self.__connect_next():
                self.quit()
                return
        self._initialize_threads()
        for _, thread in self._threads.items():
            thread.start()
        self.__connect_button.config(text="Connected")
        self._connect_post()
    def __connect_next(self):
        while (self.__hamster_comm is None or
               len(self.__hamster_comm.robotList) <= len(self._robots)):
            message = "Please connect the {} robot".format(ordinal(len(self._robots) + 1))
            if not tkMessageBox.askokcancel("Robot Connection Manager", message):
                return self.__add_virtual_substitute()
            while self.__hamster_comm is None:
                if not self.__start_hamster_comm():
                    return False
        next_hamster = self.__hamster_comm.robotList[len(self._robots)]
        self._robots.append(Robot(next_hamster, next(self.__virtual_robot_generator)))
        return True
    def __start_hamster_comm(self):
        self.__hamster_comm = HamsterComm(self._num_robots, MIN_RSSI)
        if not self.__hamster_comm.start():
            self.__hamster_comm = None
            if not tkMessageBox.askretrycancel("Robot Connection Manager",
                                               "Cannot start the robot connection "
                                               "manager. Please try again."):
                self.quit()
                return False
        return True
    def __add_virtual_substitute(self):
        if not tkMessageBox.askokcancel("Robot Connection Manager",
                                        "Add virtual robot instead?"):
            return False
        self._robots.append(Robot(None, next(self.__virtual_robot_generator)))
        return True

    # Abstract methods
    def _initialize_threads(self):
        """Register Reactors into self._threads and set up message-passing."""
        pass
    def _connect_post(self):
        """Executes after robots are connected."""
        pass
    def _generate_virtual_robots(self):
        """A generator to yield the specified number of virtual robots
        The order of virtual robots generated should correspond to the order of
        hamster robots added."""
        for _ in range(0, self._num_robots):
            yield None

class Simulator(RobotApp):
    """Displays and simulates the virtual world of a robot."""
    def __init__(self, name="Simulator", update_interval=10, num_robots=1):
        super(Simulator, self).__init__(name, update_interval, num_robots)
        self.__canvas = None
        self.__bounds = None
        self.__scale = 1
        self.__reset_button = None
        self._world = None

    # Utility for subclasses
    def _initialize_simulator_widgets(self, parent, bounds, scale=1):
        """Add RobotSimulator widgets into the specified parent widget.

        Widgets:
            canvas: canvas displaying the virtual world.
            reset: a button to reset the virtual world to its initial state.
            bounds: a 4-tuple of the min x coord, min y coord, max x coord, and
            max y coord of the virtual world that the canvas should be able to display.
            scale: number of pixels per cm of the virtual world.
        """
        toolbar_frame = ttk.Frame(parent)
        toolbar_frame.pack(side="top", fill="x")
        self.__reset_button = ttk.Button(toolbar_frame, name="reset", text="Setup",
                                         command=self._reset_simulator)
        self.__reset_button.pack(side="left")
        self.__run_button = ttk.Button(toolbar_frame, name="run", text="Start",
                                       command=self._run_simulator)
        self.__run_button.pack(side="left")

        scaled_bounds = scale_bounds(bounds, scale)
        canvas_frame = ttk.Frame(parent)
        canvas_frame.pack(fill="both", expand="yes")
        self.__canvas = tk.Canvas(canvas_frame, name="canvas", bg="white",
                                  width=scaled_bounds[2] - scaled_bounds[0],
                                  height=scaled_bounds[3] - scaled_bounds[1],
                                  scrollregion=(scaled_bounds[0], -scaled_bounds[3],
                                                scaled_bounds[2], -scaled_bounds[1]))
        horiz_scroll = AutoScrollbar(canvas_frame, orient="horizontal")
        horiz_scroll.pack(side="bottom", fill="x")
        horiz_scroll.config(command=self.__canvas.xview)
        vert_scroll = AutoScrollbar(canvas_frame, orient="vertical")
        vert_scroll.pack(side="right", fill="y")
        vert_scroll.config(command=self.__canvas.yview)
        self.__canvas.config(xscrollcommand=horiz_scroll.set,
                             yscrollcommand=vert_scroll.set)
        self.__canvas.pack(side="left", fill="both", expand="yes")
        self.__bounds = bounds
        self.__scale = scale
    def _initialize_world(self, grid_spacing=1):
        self._world = VirtualWorld("Virtual World", self.__bounds, self.__canvas,
                                   self.__scale)
        # TODO: add _world to the list of threads
        self._world.draw_grid(grid_spacing)
        self._populate_world()

    # Reset button callback
    def _reset_simulator(self):
        """(Re)initializes the simulator to its initial state."""
        if self.__reset_button.cget("text") == "Setup":
            self.__reset_button.config(text="Reset")
        self._world.reset()

    # Run button callback
    def _run_simulator(self):
        """Runs, pauses, or continues the simulator depending on its state."""
        # TODO: implement
        pass

    # Abstract methods
    def _populate_world(self):
        """Adds items to the virtual world."""
        pass
