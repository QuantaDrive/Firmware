import threading
import time
from enum import IntEnum
from typing import Tuple, Literal, List, Dict

from inputs import DeviceManager, WIN, EVENT_TYPES, EVENT_MAP

from controller import MoveSettings
from endpoints.base_jog_controller import BaseJogController

# inputs DeviceManager with refresh support
class DeviceManagerRefresh(DeviceManager):
    def __init__(self):
        self.codes = {key: dict(value) for key, value in EVENT_MAP}
        self.codes['type_codes'] = {value: key for key, value in EVENT_TYPES}
        self._raw = []
        self.keyboards = []
        self.mice = []
        self.gamepads = []
        self.other_devices = []
        self.all_devices = []
        self.leds = []
        self.microbits = []
        self.xinput = None
        self.xinput_dll = None
        if WIN:
            self._raw_device_counts = {
                'mice': 0,
                'keyboards': 0,
                'otherhid': 0,
                'unknown': 0
            }

    def refresh(self):
        self._raw = []
        self.keyboards = []
        self.mice = []
        self.gamepads = []
        self.other_devices = []
        self.all_devices = []
        self.leds = []
        self.microbits = []
        self.xinput = None
        self.xinput_dll = None
        if WIN:
            self._raw_device_counts = {
                'mice': 0,
                'keyboards': 0,
                'otherhid': 0,
                'unknown': 0
            }
        self._post_init()

class Gamepad(BaseJogController):
    class Buttons(IntEnum):
        JOYSTICK_A_X = 0
        JOYSTICK_A_Y = 1
        JOYSTICK_B_X = 2
        JOYSTICK_B_Y = 3
        TRIGGER_LEFT = 4
        TRIGGER_RIGHT = 5
        BUMPER_LEFT = 6
        BUMPER_RIGHT = 7
        DPAD_X = 8
        DPAD_Y = 9
        BUTTON_X = 10
        BUTTON_Y = 11
        BUTTON_A = 12
        BUTTON_B = 13
        MENU_LOGO = 14
        MENU_SELECT = 15
        MENU_START = 16

    def __init__(self, controls_mapping: List[Dict[str, str]]):
        self.deadzone = 3200
        self._button_values = [0] * 17
        self._controls_mapping_axis_name = controls_mapping
        self._controls_mapping_axis_id = []
        threading.Thread(target=self._poll_worker, daemon=True).start()

    def __repr__(self):
        return f"Gamepad(joystick_A={self._button_values[0:2]}, joystick_B={self._button_values[2:4]}, triggers={self._button_values[4:6]}, bumper={self._button_values[6:8]}, dpad={self._button_values[8:10]}, buttons={self._button_values[10:]})"

    def config_coordinate_names(self, coordinate_names: Tuple[str]):
        self._controls_mapping_axis_id = [0] * len(coordinate_names)
        for axis in coordinate_names:
            for mapping in self._controls_mapping_axis_name:
                if axis.upper() in mapping:
                    self._controls_mapping_axis_id[coordinate_names.index(axis)] = self.Buttons[mapping[axis.upper()].upper()]

    def get_button_value(self, button_id: Buttons):
        return self._button_values[button_id]

    def get_move(self):
        return [self._button_values[button_id] for button_id in self._controls_mapping_axis_id]

    def _poll_worker(self):
        device_manager = DeviceManagerRefresh()
        while True:
            devices_detected = False
            while not devices_detected:
                time.sleep(0.5)
                device_manager.refresh()
                devices_detected = len(device_manager.gamepads) > 0
            while devices_detected:
                try:
                    gamepad = device_manager.gamepads[0]
                    events = gamepad.read()
                except (IndexError, OSError):
                    devices_detected = False
                    continue
                for event in events:
                    if event.ev_type == 'Key':
                        if event.code == 'BTN_TL':
                            self._button_values[self.Buttons.BUMPER_LEFT] = event.state
                        elif event.code == 'BTN_TR':
                            self._button_values[self.Buttons.BUMPER_RIGHT] = event.state
                        elif event.code == 'BTN_NORTH':
                            self._button_values[self.Buttons.BUTTON_X] = event.state
                        elif event.code == 'BTN_WEST':
                            self._button_values[self.Buttons.BUTTON_Y] = event.state
                        elif event.code == 'BTN_SOUTH':
                            self._button_values[self.Buttons.BUTTON_A] = event.state
                        elif event.code == 'BTN_EAST':
                            self._button_values[self.Buttons.BUTTON_B] = event.state
                        elif event.code == 'BTN_MODE':
                            self._button_values[self.Buttons.MENU_LOGO] = event.state
                        elif event.code == 'BTN_SELECT':
                            self._button_values[self.Buttons.MENU_SELECT] = event.state
                        elif event.code == 'BTN_START':
                            self._button_values[self.Buttons.MENU_START] = event.state
                    elif event.ev_type == 'Absolute':
                        value = event.state
                        if event.code == 'ABS_HAT0X':
                            self._button_values[self.Buttons.DPAD_X] = value
                            continue
                        elif event.code == 'ABS_HAT0Y':
                            self._button_values[self.Buttons.DPAD_Y] = - value
                            continue
                        if event.code.endswith(('X', 'Y')):
                            if abs(value) < self.deadzone:
                                value = 0
                            else:
                                if value < 0:
                                    value = (value + self.deadzone) / (32768 - self.deadzone)
                                else:
                                    value = (value - self.deadzone) / (32767 - self.deadzone)
                        elif event.code.endswith('Z'):
                            value = value / 1023
                        if event.code == 'ABS_X':
                            self._button_values[self.Buttons.JOYSTICK_A_X] = value
                        elif event.code == 'ABS_Y':
                            self._button_values[self.Buttons.JOYSTICK_A_Y] = - value
                        elif event.code == 'ABS_RX':
                            self._button_values[self.Buttons.JOYSTICK_B_X] = value
                        elif event.code == 'ABS_RY':
                            self._button_values[self.Buttons.JOYSTICK_B_Y] = - value
                        elif event.code == 'ABS_Z':
                            self._button_values[self.Buttons.TRIGGER_LEFT] = value
                        elif event.code == 'ABS_RZ':
                            self._button_values[self.Buttons.TRIGGER_RIGHT] = value

class GamepadJogController(MoveSettings):
    jog_controller: Literal["Gamepad"]

    def get_jog_controller(self):
        return Gamepad(self.jog_controller_mapping)


if __name__ == "__main__":
    gamepad = Gamepad([])
    while True:
        print(gamepad)
        time.sleep(0.5)