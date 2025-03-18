from __future__ import annotations

import os
import threading
import queue
import time
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from enum import IntEnum

import yaml
from typing import List, Optional, Dict, Tuple

import serial
from pydantic import BaseModel, Field, PrivateAttr, field_validator

from kinematics import kinematic_types
from endpoints import jog_controller_types
from controller import Stepper
from controller.pin import (pin_types, pin_set_lookup_tables, validate_pin_type,
                            Multiplexer, ShiftRegister, Direction, ManualPin)


class Controller(BaseModel):
    @dataclass(order=True)
    class Command:
        priority: int
        command: bytearray = dataclass_field(compare=False)

    class MoveMode(IntEnum):
        REALTIME = 0
        CACHED = 1

    serial_mcu : Optional[str] = Field(default=None)
    kinematic_settings: kinematic_types = Field(..., alias='kinematics', discriminator="type")
    multiplexers: List[Multiplexer] = Field(default=[], max_length=4)
    shift_registers: List[ShiftRegister] = Field(default=[], max_length=4)
    steppers: List[Stepper] = Field(default=[])
    manual_pins: List[ManualPin] = Field(default=[])

    move_settings: jog_controller_types = Field(..., discriminator="jog_controller")

    _command_queue: queue.PriorityQueue = PrivateAttr()
    _move_queue: queue.Queue = PrivateAttr()
    _move_mode: MoveMode = PrivateAttr(default=MoveMode.CACHED)
    _move_thread_pause: bool = PrivateAttr(default=False)

    @field_validator("multiplexers", mode="after")
    @classmethod
    def _validate_multiplexers(cls, value: List):
        multiplexers: Dict[str, Tuple[int, Direction]] = {}
        for i in range(len(value)):
            value[i].id = i
            multiplexers[value[i].name] = (i, value[i].direction)
        pin_set_lookup_tables(multiplexers=multiplexers)
        return value

    @field_validator("shift_registers", mode="after")
    @classmethod
    def _validate_shift_registers(cls, value: List):
        shift_registers: Dict[str, int] = {}
        for i in range(len(value)):
            value[i].id = i
            shift_registers[value[i].name] = i
        pin_set_lookup_tables(shift_registers=shift_registers)
        return value

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

    # Reset controller
    def reset(self):
        command: bytearray = bytearray()
        command += b'\x00'
        command += b'\x00'
        self._command_queue.put(self.Command(0, command))
    # Actions
    def home_steppers(self):
        command_id = next(self._command_id)
        command: bytearray = bytearray()
        command += b'\x10'
        command += (command_id << 4).to_bytes(1, "big")
        self._move_queue.put(command)
        return command_id

    def home_stepper(self, stepper_id: int):
        command_id = next(self._command_id)
        command: bytearray = bytearray()
        command += b'\x11'
        command += (command_id << 4 | stepper_id).to_bytes(1, "big")
        self._move_queue.put(command)
        return command_id

    def move_steppers(self, new_positions: List[float], time: float, relative: bool = False):
        command_id = next(self._command_id)
        command: bytearray = bytearray()
        command += b'\x12'
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
        command += b'\x13'
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

    def set_gpio(self, gpio_id: int, value: bool):
        command_id = next(self._command_id)
        command: bytearray = bytearray()
        command += b'\x15'
        command += (command_id << 4).to_bytes(1, "big")

    def disable_steppers(self):
        command: bytearray = bytearray()
        command += b'\x16'
        command += b'\x00'
        self._move_queue.put(command)

    def force_stop(self):
        command: bytearray = bytearray()
        command += b'\x17'
        command += b'\x00'
        self._command_queue.put(self.Command(0, command))
    # Overrides
    def set_homed(self, stepper_id: int):
        command: bytearray = bytearray()
        command += b'\x18'
        command += stepper_id.to_bytes(1, "big")
        self._command_queue.put(self.Command(3, command))

    def set_position(self, stepper_id: int, position: float):
        command: bytearray = bytearray()
        command += b'\x19'
        command += stepper_id.to_bytes(1, "big")
        command += self.steppers[stepper_id].calculate_position(position).to_bytes(2, "big")
        self._command_queue.put(self.Command(3, command))
    # Config
    def send_config(self):
        for i in range(len(self.multiplexers)):
            command = self.multiplexers[i].get_config()
            self._command_queue.put(self.Command(1, command))
        for i in range(len(self.shift_registers)):
            command = self.shift_registers[i].get_config()
            self._command_queue.put(self.Command(1, command))
        for i in range(len(self.manual_pins)):
            commands = self.manual_pins[i].get_config()
            self._command_queue.put(self.Command(1, commands[0]))
            if len(commands) > 1:
                self._command_queue.put(self.Command(2, commands[1]))
        for i in range(len(self.steppers)):
            for command in self.steppers[i].get_config(i):
                self._command_queue.put(self.Command(1, command))

    def set_move_mode(self, mode: Controller.MoveMode):
        self._move_thread_pause = True
        self._move_mode = mode
        while not self._move_queue.empty():
            self._move_queue.get(block=False)
            self._move_queue.task_done()
        if mode == Controller.MoveMode.CACHED:
            self._move_queue.maxsize = 0
        elif mode == Controller.MoveMode.REALTIME:
            self._move_queue.maxsize = 1
        command: bytearray = bytearray()
        command += b'\x3C'
        command += mode.to_bytes(1, "big")
        self._command_queue.put(self.Command(0, command))
        self._move_thread_pause = False

    def _send_command_worker(self):
        with serial.Serial(self.serial_mcu, 115200) as connection:
            transmit_move: bool = False
            last_keep_alive: float = time.time()
            while True:
                while self._move_thread_pause:
                    time.sleep(0.25)
                command_out = None
                config_command = False
                move_command = False
                if not self._command_queue.empty():
                    command_out = self._command_queue.get().command
                    config_command = True
                elif transmit_move and not self._move_queue.empty():
                    command_out = self._move_queue.get()
                    transmit_move = False
                    move_command = True
                elif time.time() - last_keep_alive > 1:
                    command_out = b'\xEF'
                    command_out += b'\x00'
                if command_out is not None:
                    connection.write(command_out)
                    if os.environ.get("MCU_DEBUG") is not None:
                        print(str(command_out).replace('\\x', ' ').replace('\\r', ' 0d').replace('\\n', ' 0a'))
                    op_code_check = connection.read(1)
                    while op_code_check != command_out[0:1]:
                        if op_code_check == b'\xFF':
                            transmit_move = True
                        else:
                            print("Error:", str(op_code_check) + " != " + str(command_out[0:1]))
                            print("Command:", str(command_out).replace('\\x', ' ').replace('\\r', ' 0d').replace('\\n', ' 0a'))
                            print("Trying again...")
                        op_code_check = connection.read(1)
                    last_keep_alive = time.time()
                    if config_command:
                        self._command_queue.task_done()
                    if move_command:
                        self._move_queue.task_done()
                else:
                    if transmit_move:
                        # print("Waiting for move...")
                        if self._move_mode == Controller.MoveMode.REALTIME:
                            time.sleep(0.025)
                        elif self._move_mode == Controller.MoveMode.CACHED:
                            time.sleep(0.1)
                    else:
                        # print("Waiting for move transmit request...")
                        transmit_move_check = connection.read(1)
                        if transmit_move_check == b'\xFF':
                            transmit_move = True
                        last_keep_alive = time.time()
