import atexit
import logging
import os
import select
import sys
import termios
import time
import tty
from queue import Queue
from threading import Thread

logger = logging.getLogger(__name__)


class KeyboardThread(Thread):
    def __init__(self, event_queue: Queue):
        super().__init__(name="KeyboardThread", daemon=True)
        self.event_queue = event_queue

    def run(self):
        if os.environ.get("DISPLAY"):
            try:
                self._run_pynput()
                return
            except Exception as exc:
                logger.warning("pynput keyboard backend failed, falling back to terminal input: %s", exc)

        self._run_terminal()

    def _run_pynput(self):
        from pynput import keyboard

        def on_press(key):
            key_name = self.get_key_name(key)
            event = {
                "type": "keyboard",
                "name": key_name,
                "pressed": True,
                "timestamp": time.time(),
                "source": "pynput",
            }
            self.event_queue.put(event)

        def on_release(key):
            key_name = self.get_key_name(key)
            event = {
                "type": "keyboard",
                "name": key_name,
                "pressed": False,
                "timestamp": time.time(),
                "source": "pynput",
            }
            self.event_queue.put(event)

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    def _run_terminal(self):
        if not sys.stdin.isatty():
            logger.warning("KeyboardCtrl needs a TTY or DISPLAY; no keyboard events will be captured.")
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        def restore_terminal():
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except termios.error:
                pass

        atexit.register(restore_terminal)
        tty.setcbreak(fd)
        logger.info("KeyboardCtrl using terminal input backend")
        try:
            while True:
                readable, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not readable:
                    continue
                key_name = self.get_terminal_key_name(sys.stdin.read(1))
                if key_name is None:
                    continue
                self.event_queue.put(
                    {
                        "type": "keyboard",
                        "name": key_name,
                        "pressed": True,
                        "timestamp": time.time(),
                        "source": "terminal",
                    }
                )
        finally:
            restore_terminal()

    def get_key_name(self, key):
        try:
            return key.char if key.char is not None else str(key)
        except AttributeError:
            return str(key)

    def get_terminal_key_name(self, char: str):
        match char:
            case "\x1b":
                return "Key.esc"
            case "\r" | "\n":
                return "Key.enter"
            case "\t":
                return "Key.tab"
            case " ":
                return "Key.space"
            case "\x03":
                return "Key.ctrl_c"
            case "":
                return None
            case _:
                return char


if __name__ == "__main__":
    event_queue = Queue()
    kb_thread = KeyboardThread(event_queue)
    kb_thread.start()

    print("Press keys (ESC to exit)...")
    while True:
        event = event_queue.get()
        print(event)
        if event["name"] == "Key.esc" and event["pressed"]:
            break
