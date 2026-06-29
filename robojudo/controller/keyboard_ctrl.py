import time
from queue import Empty, Queue

from robojudo.controller import Controller, ctrl_registry
from robojudo.controller.ctrl_cfgs import KeyboardCtrlCfg
from robojudo.controller.utils.keyboard import KeyboardThread


@ctrl_registry.register
class KeyboardCtrl(Controller):
    cfg_ctrl: KeyboardCtrlCfg

    def __init__(self, cfg_ctrl: KeyboardCtrlCfg, env=None, **kwargs):  # TODO
        super().__init__(cfg_ctrl=cfg_ctrl, env=env, **kwargs)

        self.event_queue = Queue(maxsize=100)
        self.keys_pressed: dict[str, float] = {}
        self.key_sources: dict[str, str] = {}
        self.keyboard_thread = KeyboardThread(self.event_queue)
        self.keyboard_thread.start()

        self.reset()

    def reset(self):
        while not self.event_queue.empty():
            try:
                self.event_queue.get_nowait()
            except Empty:
                break
        self.keys_pressed.clear()
        self.key_sources.clear()

    def get_events(self):
        events = []
        while not self.event_queue.empty():
            try:
                event = self.event_queue.get_nowait()
                events.append(event)
            except Empty:
                break
        return events

    def get_data(self):
        events = self.get_events()
        now = time.time()
        for event in events:
            if event["type"] != "keyboard":
                continue

            key_name = event["name"]
            if event["pressed"]:
                if key_name == "Key.space":
                    self.keys_pressed.clear()
                    self.key_sources.clear()
                    continue
                self.keys_pressed[key_name] = event["timestamp"]
                self.key_sources[key_name] = event.get("source", "pynput")
            else:
                self.keys_pressed.pop(key_name, None)
                self.key_sources.pop(key_name, None)

        for key_name, timestamp in list(self.keys_pressed.items()):
            if self.key_sources.get(key_name) == "terminal" and now - timestamp > self.cfg_ctrl.terminal_key_timeout:
                self.keys_pressed.pop(key_name, None)
                self.key_sources.pop(key_name, None)

        return {
            "keyboard_event": events,
            "keys_pressed": list(self.keys_pressed.keys()),
        }

    def process_triggers(self, ctrl_data):
        commands = []
        if len(self.triggers) == 0:
            return ctrl_data, commands

        for event in ctrl_data["keyboard_event"]:
            if event["type"] != "keyboard":
                continue

            # pynput provides release events; terminal input does not, so trigger
            # terminal commands on press.
            should_trigger = not event["pressed"] or event.get("source") == "terminal"
            if should_trigger:
                command = self.triggers.get(event["name"], None)
                if command is not None:
                    commands.append(command)
                    # remove event after triggered
                    ctrl_data["keyboard_event"].remove(event)

        return ctrl_data, commands


if __name__ == "__main__":
    kb_ctrl = KeyboardCtrl(
        cfg_ctrl=KeyboardCtrlCfg(
            triggers={
                "Key.space": "[TEST]",
                "\x01": "[CTRL_A]",
            }
        )
    )
    while True:
        data = kb_ctrl.get_data()
        ctrl_data, commands = kb_ctrl.process_triggers(data)
        if ctrl_data["keyboard_event"]:
            for e in ctrl_data["keyboard_event"]:
                print(e)
        if commands:
            print("Commands:", commands)
        time.sleep(0.1)
