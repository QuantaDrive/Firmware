import os
import threading
import queue
from dataclasses import dataclass
from dataclasses import field as dataclass_field

import yaml
from typing import List, Optional, Tuple

import serial
from pydantic import BaseModel, Field, PrivateAttr

from gpio import Gpio
from stepper import Stepper

def load_controller_config(config_file):
    with open(config_file, "r") as f:
        return Controller(**yaml.safe_load(f))

@dataclass(order=True)
class Command:
    priority: int
    command: bytearray = dataclass_field(compare=False)

class Controller(BaseModel):
    serial_mcu : Optional[str] = Field(default=None)
    steppers: Optional[List[Stepper]] = Field(default=None)
    gpios: Optional[List[Gpio]] = Field(default=None)

    start_velocity: float = Field(default=1)     # mm per second
    max_velocity: float = Field(default=50)      # mm per second
    max_accel: float = Field(default=10)         # mm per second^2

    _command_queue: queue.PriorityQueue = PrivateAttr()
    _move_queue: queue.Queue = PrivateAttr()
    _coordinates: Tuple[float] = PrivateAttr()

    def __init__(self, **data):
        super().__init__(**data)
        for stepper in self.steppers:
            stepper.max_velocity = self.max_velocity

    @property
    def coordinates(self):
        return self._coordinates

    @coordinates.setter
    def coordinates(self, coordinates):
        self._coordinates = coordinates

    def find_device(self):
        devices = []
        if self.serial_mcu is not None:
            if not os.path.exists(self.serial_mcu):
                print(f"Device {self.serial_mcu} not found.")
                exit(1)
            return
        for device in os.listdir("/dev/serial/by-id"):
            if device.startswith("usb-Raspberry_Pi_Pico"):
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
        self._command_queue = queue.PriorityQueue(10)
        self._move_queue = queue.Queue(50)
        threading.Thread(target=self.send_command_worker, daemon=True).start()

    def send_config(self):
        for i in range(len(self.steppers)):
            for command in self.steppers[i].get_config(i):
                self._command_queue.put(Command(2, command))

    def reset(self):
        command: bytearray = bytearray()
        command += b'\x00'
        self._command_queue.put(Command(0, command))

    def force_stop(self):
        command: bytearray = bytearray()
        command += b'\x18'
        self._command_queue.put(Command(0, command))

    def move_steppers(self, new_positions: List[float], time: float, relative: bool = False):
        command: bytearray = bytearray()
        command += b'\x10'
        for i in range(len(self.steppers)):
            command += b'\x00'
            command += self.steppers[i].move(new_positions[i], time, relative)
        self._move_queue.put(command)

    def move_stepper(self, stepper_id: int, new_position: float, time: float, relative: bool = False):
        command: bytearray = bytearray()
        command += b'\x11'
        command += stepper_id.to_bytes(1, "big")
        command += self.steppers[stepper_id].move(new_position, time, relative)
        self._move_queue.put(command)

    def send_command_worker(self):
        connection = serial.Serial(self.serial_mcu, 115200, timeout=0.1)
        transmit_request: bool = True
        while True:
            if connection.in_waiting > 0:
                command_in = connection.read(1)
                if command_in == b'\x00':
                    transmit_request = True
                elif command_in == b'\xFF':
                    print("Error")
            if transmit_request:
                if not self._command_queue.empty():
                    command_out = self._command_queue.get().command
                    transmit_request = False
                    connection.write(command_out)
                    self._command_queue.task_done()
                elif not self._move_queue.empty():
                    command_out = self._move_queue.get()
                    transmit_request = False
                    connection.write(command_out)
                    self._move_queue.task_done()
