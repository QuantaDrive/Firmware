from enum import IntEnum
from functools import cached_property
from typing import Optional, List

import numpy as np
from pydantic import BaseModel, Field, PositiveFloat, field_validator, PositiveInt, computed_field, PrivateAttr

from gpio import Gpio, Direction

class DriverType(IntEnum):
    GENERIC = 1
    TMC2240 = 2
    TMC5160 = 3

class Driver(BaseModel):
    type: DriverType = Field(default=DriverType.GENERIC)
    step_pin: Gpio
    direction_pin: Gpio #IMPORTANT this pin has to be unique because the mcu reads the current state of the pin to see direction
    enable_pin: Optional[Gpio] = Field(default=None)
    diag_fault_pin: Optional[Gpio] = Field(default=None)
    spi_cs_pin: Optional[Gpio] = Field(default=Gpio())

    @field_validator("type", mode="before")
    @classmethod
    def _validate_type(cls, value):
        if isinstance(value, str):
            return DriverType[value.upper()]
        return value

    @field_validator("step_pin", "direction_pin",
                     "enable_pin", "spi_cs_pin", mode="after")
    @classmethod
    def _validate_output_pins(cls, value: Gpio):
        value.direction = Direction.OUTPUT
        return value

    @field_validator("diag_fault_pin", mode="after")
    @classmethod
    def _validate_input_pins(cls, value: Gpio):
        value.direction = Direction.INPUT
        return value

    def get_config(self, stepper_id: int) -> List[bytearray]:
        commands = []
        set_step_dir_command: bytearray = bytearray()
        set_step_dir_command += b"\x01"
        set_step_dir_command += stepper_id.to_bytes(1, "big")
        set_step_dir_command += self.step_pin.pin_number_config
        set_step_dir_command += self.direction_pin.pin_number_config
        commands.append(set_step_dir_command)
        if self.enable_pin is not None and self.diag_fault_pin is not None:
            set_enable_fault_command: bytearray = bytearray()
            set_enable_fault_command += b"\x02"
            set_enable_fault_command += stepper_id.to_bytes(1, "big")
            set_enable_fault_command += self.enable_pin.pin_number_config
            set_enable_fault_command += self.diag_fault_pin.pin_number_config
            commands.append(set_enable_fault_command)
        set_driver_cs_command: bytearray = bytearray()
        set_driver_cs_command += b"\x03"
        set_driver_cs_command += stepper_id.to_bytes(1, "big")
        set_driver_cs_command += self.type.to_bytes(1, "big")
        set_driver_cs_command += self.spi_cs_pin.pin_number_config
        commands.append(set_driver_cs_command)
        return commands


class Stepper(BaseModel):
    driver: Driver
    mm_per_rev: PositiveFloat = Field(default=1)
    gear_ratio: PositiveFloat = Field(default=1)
    steps_per_rev: PositiveInt = Field(default=200)
    microsteps: PositiveInt = Field(default=1)
    min_position: float = Field(default=0)
    max_position: float = Field(default=np.inf)

    _current_position: float = PrivateAttr(default=0)
    _current_steps: int = PrivateAttr(default=0)

    def __init__(self, **data):
        super().__init__(**data)
        self._current_steps = round((self._current_position - self.min_position) * self.final_steps_per_mm)

    @computed_field(return_type=float)
    @cached_property
    def final_steps_per_mm(self):
        return self.gear_ratio * self.steps_per_rev * self.microsteps / self.mm_per_rev

    @computed_field(return_type=int)
    @cached_property
    def min_position_steps(self):
        return int(self.min_position * self.final_steps_per_mm)

    @computed_field(return_type=int)
    @cached_property
    def max_position_steps(self):
        if self.max_position == np.inf:
            return np.inf
        return int(self.max_position * self.final_steps_per_mm)

    def get_config(self, stepper_id: int) -> List[bytearray]:
        return self.driver.get_config(stepper_id)

    def calculate_position(self, position: float) -> int:
        if position > self.max_position or position < self.min_position:
            print("Position out of range: " + str(position))
            position = np.clip(position, self.min_position, self.max_position)
            #raise ValueError("Position out of range")
        position_normalized = position - self.min_position
        return int(round(position_normalized * self.final_steps_per_mm))

    def calculate_speed(self, position_steps: int, time: float) -> int:
        delta_steps = np.abs(position_steps - self._current_steps)
        print(delta_steps)
        if delta_steps == 0:
            return 0
        steps_per_second = delta_steps / time
        ticks_per_step = 50000 / steps_per_second
        return int(max(ticks_per_step, 1))

    def move(self, new_position: float, time: float, relative: bool = False) -> bytearray:
        if relative:
            new_position += self._current_position
        new_position_steps = self.calculate_position(new_position)
        ticks_per_step = self.calculate_speed(new_position_steps, time)
        move_data: bytearray = bytearray()
        move_data += new_position_steps.to_bytes(2, "big")
        move_data += ticks_per_step.to_bytes(3, "big")
        if ticks_per_step != 0: # Because if the speed is 0, the position will not change
            self._current_position = new_position
            self._current_steps = new_position_steps
        return move_data