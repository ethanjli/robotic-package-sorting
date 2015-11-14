"""Script to test basic functionality of GUIReactor."""
import sys
import Tkinter as tk

from components.concurrency import GUIReactor

class GUIHelloWorld(GUIReactor):
    """Shows a simple window with a hello world message and a quit button."""
    def __init__(self):
        super(GUIHelloWorld, self).__init__()
        self._message = tk.Label(self._root, text="Hello, World!")
        self._message.pack()
        self._quit_button = tk.Button(self._root, text="Quit", command=self.quit)
        self._quit_button.pack()

    def _run_pre(self):
        print("Starting...")
    def _run_post(self):
        print("Finished!")

def main():
    """Runs test."""
    gui = GUIHelloWorld()
    gui.start()

if __name__ == "__main__":
    sys.exit(main())
