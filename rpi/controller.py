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
from gpio import Gpio
from stepper import Stepper

@dataclass(order=True)
class Command:
    priority: int
    command: bytearray = dataclass_field(compare=False)

class Controller(BaseModel):
    serial_mcu : Optional[str] = Field(default=None)
    kinematic_settings: kinematic_types = Field(..., alias='kinematics', discriminator="type")
    steppers: Optional[List[Stepper]] = Field(default=None)
    gpios: Optional[List[Gpio]] = Field(default=None)

    max_velocity: float = Field(default=50)      # mm per second
    max_accel: float = Field(default=10)         # mm per second^2

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
                self._command_queue.put(Command(1, command))

    def reset(self):
        command: bytearray = bytearray()
        command += b'\x00'
        self._command_queue.put(Command(0, command))

    def force_stop(self):
        command: bytearray = bytearray()
        command += b'\x18'
        self._command_queue.put(Command(0, command))

    def set_homed(self, stepper_id: int):
        command: bytearray = bytearray()
        command += b'\x1E'
        command += stepper_id.to_bytes(1, "big")
        self._command_queue.put(Command(3, command))

    def set_position(self, stepper_id: int, position: float):
        command: bytearray = bytearray()
        command += b'\x1F'
        command += stepper_id.to_bytes(1, "big")
        command += self.steppers[stepper_id].calculate_position(position).to_bytes(2, "big")
        self._command_queue.put(Command(3, command))

    def home_steppers(self):
        command: bytearray = bytearray()
        command += b'\x14'
        self._move_queue.put(command)

    def home_stepper(self, stepper_id: int):
        command: bytearray = bytearray()
        command += b'\x15'
        command += stepper_id.to_bytes(1, "big")
        self._move_queue.put(command)

    def move_steppers(self, new_positions: List[float], time: float, relative: bool = False):
        command: bytearray = bytearray()
        command += b'\x10'
        for i in range(len(self.steppers)):
            command += b'\x00'
            command += self.steppers[i].move(new_positions[i], time, relative)
        self._move_queue.put(command)

    def move_steppers_interpolated(self, new_positions: List[float], time: float, relative: bool = False):
        stepper_move_generators = []
        for i in range(len(self.steppers)):
            stepper_move_generators.append(self.steppers[i].interpolate_position(new_positions[i], time, 0.1, relative))
        for positions in zip(*stepper_move_generators):
            self.move_steppers(positions, 0.1, relative)


    def move_stepper(self, stepper_id: int, new_position: float, time: float, relative: bool = False):
        command: bytearray = bytearray()
        command += b'\x11'
        command += stepper_id.to_bytes(1, "big")
        command += self.steppers[stepper_id].move(new_position, time, relative)
        self._move_queue.put(command)

    def _send_command_worker(self):
        with serial.Serial(self.serial_mcu, 115200) as connection:
            transmit_request: bool = True
            transmit_move: bool = False
            last_keep_alive: float = time.time()
            while True:
                if time.time() - last_keep_alive > 0.01:
                    connection.write(b'\xFF')
                    check = connection.read(1)
                    if check == b'\xFF':
                        last_keep_alive = time.time()
                    else:
                        print("Connection timed out to MCU.")

                # if time.time() - last_keep_alive > 5:
                #     raise TimeoutError("Connection timed out to MCU.")
                if connection.in_waiting > 0:
                    last_keep_alive = time.time()
                    command_in = connection.read(1)
                    if command_in == b'\x00':
                        transmit_request = True
                    elif command_in == b'\x01':
                        transmit_request = True
                        transmit_move = True
                if transmit_request:
                    command_out = None
                    move_command = True
                    if not self._command_queue.empty():
                        command_out = self._command_queue.get().command
                        move_command = False
                    elif transmit_move and not self._move_queue.empty():
                        command_out = self._move_queue.get()
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
                        if move_command:
                            self._move_queue.task_done()
                        else:
                            self._command_queue.task_done()
                        if os.environ.get("MCU_DEBUG") is not None:
                            print(str(command_out).replace('\\x', ' ').replace('\\r', ' 0d').replace('\\n', ' 0a'))
