import os
import threading
import queue
import time
from dataclasses import dataclass
from dataclasses import field as dataclass_field

import yaml
from typing import List, Optional

import serial
from pydantic import BaseModel, Field, PrivateAttr

from kinematics import kinematic_types
from endpoints import jog_controller_types
from controller import Gpio, Stepper


class Controller(BaseModel):
    @dataclass(order=True)
    class Command:
        priority: int
        command: bytearray = dataclass_field(compare=False)

    serial_mcu : Optional[str] = Field(default=None)
    kinematic_settings: kinematic_types = Field(..., alias='kinematics', discriminator="type")
    steppers: Optional[List[Stepper]] = Field(default=None)
    gpios: Optional[List[Gpio]] = Field(default=None)

    move_settings: jog_controller_types = Field(..., discriminator="jog_controller")

    _command_queue: queue.PriorityQueue = PrivateAttr()
    _move_queue: queue.Queue = PrivateAttr()

    def __init__(self, **data):
        super().__init__(**data)
        self._command_queue = queue.PriorityQueue(10)
        if os.environ.get("MCU_DEBUG") is not None:
            self._move_queue = queue.Queue(1)
        else:
            self._move_queue = queue.Queue()

    @classmethod
    def from_config(cls, config_file):
        with open(config_file, "r") as f:
            return cls(**yaml.safe_load(f))

    @property
    def _command_id(self):
        command_id = 0
        while True:
            yield command_id
            command_id += 1
            command_id %= 16


    def find_device(self):
        device_prefix = "usb-Raspberry_Pi_Pico"
        if os.environ.get("MCU_DEBUG") is not None:
            device_prefix = "usb-Raspberry_Pi_Debug_Probe__CMSIS-DAP"
        devices = []
        if self.serial_mcu is not None:
            if not os.path.exists(self.serial_mcu):
                print(f"Device {self.serial_mcu} not found.")
                exit(1)
            return

        if os.path.exists("/dev/serial/by-id/"):
            for device in os.listdir("/dev/serial/by-id"):
                if device.startswith(device_prefix):
                    devices.append("/dev/serial/by-id/" + device)

        if len(devices) == 0:
            print("No devices found")
            exit(1)
        elif len(devices) == 1:
            self.serial_mcu = devices[0]
            return
        else:
            print("Multiple devices found.")
            print("Please provide one in the config file with the serial_mcu field.")
            exit(1)

    def connect(self):
        self.find_device()
        threading.Thread(target=self._send_command_worker, daemon=True).start()

    def send_config(self):
        for i in range(len(self.steppers)):
            for command in self.steppers[i].get_config(i):
                self._command_queue.put(self.Command(1, command))

    def reset(self):
        command: bytearray = bytearray()
        command += b'\x00'
        command += b'\x00'
        self._command_queue.put(self.Command(0, command))

    def force_stop(self):
        command: bytearray = bytearray()
        command += b'\x18'
        command += b'\x00'
        self._command_queue.put(self.Command(0, command))

    def set_homed(self, stepper_id: int):
        command: bytearray = bytearray()
        command += b'\x1E'
        command += stepper_id.to_bytes(1, "big")
        self._command_queue.put(self.Command(3, command))

    def set_position(self, stepper_id: int, position: float):
        command: bytearray = bytearray()
        command += b'\x1F'
        command += stepper_id.to_bytes(1, "big")
        command += self.steppers[stepper_id].calculate_position(position).to_bytes(2, "big")
        self._command_queue.put(self.Command(3, command))

    def home_steppers(self):
        command_id = next(self._command_id)
        command: bytearray = bytearray()
        command += b'\x14'
        command += (command_id << 4).to_bytes(1, "big")
        self._move_queue.put(command)
        return command_id

    def home_stepper(self, stepper_id: int):
        command_id = next(self._command_id)
        command: bytearray = bytearray()
        command += b'\x15'
        command += (command_id << 4 | stepper_id).to_bytes(1, "big")
        self._move_queue.put(command)
        return command_id

    def move_steppers(self, new_positions: List[float], time: float, relative: bool = False):
        command_id = next(self._command_id)
        command: bytearray = bytearray()
        command += b'\x10'
        command += (command_id << 4).to_bytes(1, "big")
        for i in range(len(self.steppers)):
            command += self.steppers[i].move(new_positions[i], time, relative)
        self._move_queue.put(command)
        return command_id

    def move_steppers_interpolated(self, new_positions: List[float], time: float, relative: bool = False):
        stepper_move_generators = []
        for i in range(len(self.steppers)):
            stepper_move_generators.append(self.steppers[i].interpolate_position(new_positions[i], time, 0.1, relative))
        for positions in zip(*stepper_move_generators):
            self.move_steppers(positions, 0.1, relative)

    def move_stepper(self, stepper_id: int, new_position: float, time: float, relative: bool = False):
        command_id = next(self._command_id)
        command: bytearray = bytearray()
        command += b'\x11'
        command += (command_id << 4 | stepper_id).to_bytes(1, "big")
        command += self.steppers[stepper_id].move(new_position, time, relative)
        self._move_queue.put(command)
        return command_id

    def dwell(self, s = 0, ms = 0):
        command_id = next(self._command_id)
        us = (s * 1e6 + ms * 1e3) / 20
        command: bytearray = bytearray()
        command += b'\x14'
        command += (command_id << 4).to_bytes(1, "big")
        command += int(us).to_bytes(3, "big")
        self._move_queue.put(command)
        return command_id

    def _send_command_worker(self):
        with serial.Serial(self.serial_mcu, 115200) as connection:
            transmit_request: bool = True
            transmit_move: bool = False
            last_keep_alive: float = time.time()
            while True:
                if transmit_request or time.time() - last_keep_alive > 1:
                    command_out = None
                    config_command = False
                    move_command = False
                    if not self._command_queue.empty():
                        command_out = self._command_queue.get().command
                        config_command = True
                    elif transmit_move and not self._move_queue.empty():
                        command_out = self._move_queue.get()
                        move_command = True
                    elif time.time() - last_keep_alive > 1:
                        command_out = b'\xFF'
                    if command_out is not None:
                        transmit_request = False
                        transmit_move = False
                        connection.write(command_out)
                        op_code_check = connection.read(1)
                        while op_code_check != command_out[0:1]:
                            print("Error:", str(op_code_check) + " != " + str(command_out[0:1]))
                            print("Command:", str(command_out).replace('\\x', ' ').replace('\\r', ' 0d').replace('\\n', ' 0a'))
                            print("Trying again...")
                            op_code_check = connection.read(1)
                        status_code = connection.read(1)
                        if status_code == b'\x00':
                            transmit_request = True
                        elif status_code == b'\x01':
                            transmit_request = True
                            transmit_move = True
                        else:
                            print("Error: ", str(status_code))
                        last_keep_alive = time.time()
                        if config_command:
                            self._command_queue.task_done()
                        if move_command:
                            self._move_queue.task_done()
                        if os.environ.get("MCU_DEBUG") is not None:
                            print(str(command_out).replace('\\x', ' ').replace('\\r', ' 0d').replace('\\n', ' 0a'))
                    else:
                        time.sleep(0.1)
                else:
                    time.sleep(0.25)
