"""Classes to setup and run graphical programs involving robots."""
import time
import Tkinter as tk
import tkMessageBox
import ttk

from components.util import ordinal
from components.messaging import Receiver, Broadcaster
from components.robots import HamsterComm, Robot
from components.world import VirtualWorld

MIN_RSSI = -60

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
        self.__connect_button.pack(side="top", fill="x")
        self.__quit_button = ttk.Button(parent, name="quit", text="Quit",
                                        command=self.quit)
        self.__quit_button.pack(side="top", fill="x")
    def _add_robot_threads(self):
        """Add the virtual robots made from the connect button to the list of threads."""
        for robot in self._robots:
            virtual = robot.get_virtual()
            if virtual is not None:
                self._add_thread(virtual)
    def _add_thread(self, thread):
        """Adds the given thread object to the list of threads managed by the app."""
        self._threads[thread.get_name()] = thread
    def _start_threads(self):
        """Starts all threads managed by the app."""
        for _, thread in self._threads.items():
            thread.start()
    def _disable_connect_button(self, new_text):
        """Disables the connect button and changes the text."""
        self.__connect_button.config(state="disabled")
        self.__connect_button.config(text=new_text)


    # Connect button callback
    def __connect_all(self):
        self._disable_connect_button("Connecting")
        while len(self._robots) < self._num_robots:
            if not self.__connect_next():
                self.quit()
                return
        self._initialize_threads()
        self._start_threads()
        self._disable_connect_button("Connected")
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
        robot = Robot(next_hamster, next(self.__virtual_robot_generator))
        self._robots.append(robot)
        self._add_robot_post(robot)
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
        robot = Robot(None, next(self.__virtual_robot_generator))
        self._robots.append(robot)
        self._add_robot_post(robot)
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
        hamster robots added.
        """
        for _ in range(0, self._num_robots):
            yield None
    def _add_robot_post(self, robot):
        """Executes after each robot (whether virtual or real) is connected."""
        pass

class Simulator(RobotApp):
    """Displays and simulates the virtual world of a robot.

    Signals Received:
        Will react to any Signal of correct name.
        UpdateCoords: Data should be a 2-tuple of the canvas item specifier and a
        tuple of the new coords.
    """
    def __init__(self, name="Simulator", update_interval=10, num_robots=1):
        super(Simulator, self).__init__(name, update_interval, num_robots)
        self.__canvas = None
        self.__bounds = None
        self.__scale = 1
        self.__reset_button = None
        self.__run_button = None
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
        # Set up toolbar
        toolbar_frame = ttk.Frame(parent)
        toolbar_frame.pack(side="top", fill="x")
        self.__reset_button = ttk.Button(toolbar_frame, name="reset", text="Setup",
                                         command=self.__reset_simulator)
        self.__reset_button.pack(side="left")
        self.__run_button = ttk.Button(toolbar_frame, name="run", text="Start",
                                       command=self.__run_simulator, state="disabled")
        self.__run_button.pack(side="left")

        # Set up canvas
        scaled_bounds = tuple(scale * bound for bound in bounds)
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

        # Set up virtual world
        self._world = VirtualWorld("Virtual World", self.__bounds, self.__canvas,
                                   self.__scale)
    def _initialize_world(self, grid_spacing=1):
        """Initializes the world and calls the _populate_world method to add objects.
        Must be called after the implementing subclass calls the _initialize_simulator_widgets
        method. Should probably be called in the implementing subclass's __init__ method.
        """
        self._world.draw_grid(grid_spacing)
        self._world.register("UpdateCoords", self)
        self._world.register("UpdateConfig", self)
        self._populate_world()
    def _add_virtual_world_threads(self):
        """Add the threads representing the virtual world and objects in it.
        Should probably be called in the implementing subclass's _initialize_threads method.
        """
        self._add_robot_threads()
        self._threads[self._world.get_name()] = self._world
    def _change_reset_button(self, new_text):
        """Changes the text (and thus state) of the reset button."""
        self.__reset_button.config(text=new_text)
    def _enable_start_button(self):
        """Enables the simulation start button.
        Should probably be called in the implementing subclass's _connect_post method.
        """
        self.__run_button.config(state="normal")

    # Reset button callback
    def __reset_simulator(self):
        """(Re)initializes the simulator to its initial state."""
        state = self.__reset_button.cget("text")
        if state == "Setup":
            self.__setup_simulator()
            self.__reset_button.config(text="Reset")
        self._world.reset()
        self.__run_button.config(text="Start")
        if not state == "Setup":
            self._reset_simulator_post()
    def __setup_simulator(self):
        """Bypasses the app's connect button to instantiate the virtual robots.
        Will call the _connect_post() method, which should add the virtual robots
        to the virtual world."""
        self._disable_connect_button("Simulating")
        virtual_robot_generator = self._generate_virtual_robots()
        for _ in range(0, self._num_robots):
            robot = Robot(None, next(virtual_robot_generator))
            self._robots.append(robot)
            self._add_robot_post(robot)
        self._initialize_threads()
        self._start_threads()
        self._connect_post()

    # Run button callback
    def __run_simulator(self):
        """Runs, pauses, or continues the simulator depending on its state."""
        state = self.__run_button.cget("text")
        if state == "Start":
            self.__run_button.config(text="Pause")
            self._start_simulator()
        elif state == "Pause":
            self.__run_button.config(text="Resume")
            self._pause_simulator()
        elif state == "Resume":
            self.__run_button.config(text="Pause")
            self._resume_simulator()

    # Implementation of parent abstract methods
    def _react(self, signal):
        if signal.Name == "UpdateCoords":
            self.__canvas.coords(signal.Data[0], *signal.Data[1])
        elif signal.Name == "UpdateConfig":
            self.__canvas.itemconfig(signal.Data[0], **signal.Data[1])
        else:
            self._react_simulator(signal)
    def _add_robot_post(self, robot):
        """Add the robot to the virtual world."""
        self._world.add_robot(robot)

    # Abstract methods
    def _start_simulator(self):
        """Starts simulation execution. Called once."""
        pass
    def _reset_simulator_post(self):
        """Called after the simulator's world is reset.
        Use this to reset the state of planners, etc."""
        pass
    def _pause_simulator(self):
        """Pauses simulation execution."""
        pass
    def _resume_simulator(self):
        """Resumes paused simulation execution."""
        pass
    def _populate_world(self):
        """Adds items to the virtual world, not including robots."""
        pass
    def _react_simulator(self, signal):
        """Reacts to any Signals not already caught by the simulator."""
        pass
