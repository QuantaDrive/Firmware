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
    direction_pin: Gpio
    enable_pin: Optional[Gpio] = Field(default=None)
    diag_fault_pin: Optional[Gpio] = Field(default=None)
    spi_cs_pin: Optional[Gpio] = Field(default=None)

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
        if self.spi_cs_pin is None:
            set_driver_cs_command += b"\x00"
        else:
            set_driver_cs_command += self.spi_cs_pin.pin_number_config
        commands.append(set_driver_cs_command)
        return commands


class Stepper(BaseModel):
    driver: Driver
    steps_per_mm: PositiveFloat = Field(default=200)
    microsteps: PositiveInt = Field(default=1)
    min_position: float = Field(default=0)
    max_position: float = Field(default=np.inf)

    _current_position: float = PrivateAttr(default=0)
    _max_velocity_steps: float = PrivateAttr(default=np.inf)

    def __init__(self, **data):
        super().__init__(**data)

    @computed_field(return_type=int)
    @cached_property
    def min_position_steps(self):
        return int(self.min_position * self.steps_per_mm * self.microsteps)

    @computed_field(return_type=int)
    @cached_property
    def max_position_steps(self):
        if self.max_position == np.inf:
            return np.inf
        return int(self.max_position * self.steps_per_mm * self.microsteps)

    # @computed_field
    @property
    def max_velocity_steps(self):
        return self._max_velocity_steps

    @max_velocity_steps.setter
    def max_velocity_steps(self, velocity: float):
        self._max_velocity_steps = velocity * self.steps_per_mm * self.microsteps

    def get_config(self, stepper_id: int) -> List[bytearray]:
        return self.driver.get_config(stepper_id)

    def calculate_position(self, position: float) -> int:
        if position > self.max_position_steps or position < self.min_position_steps:
            raise ValueError("Position out of range")
        position_normalized = position - self.min_position
        return int(round(position_normalized * self.steps_per_mm * self.microsteps))

    def calculate_speed(self, position: float, new_position: float, time: float) -> int:
        delta_position = np.abs(new_position - position)
        delta_steps = round(delta_position * self.steps_per_mm * self.microsteps)
        if delta_steps == 0:
            return 0
        steps_per_second = delta_steps / time
        if steps_per_second > self.max_velocity_steps:
            raise ValueError("Velocity too high")
        ticks_per_step = 50000 / steps_per_second
        return int(ticks_per_step)

    def move(self, new_position: float, time: float, relative: bool = False) -> bytearray:
        if relative:
            new_position += self._current_position
        new_position_steps = self.calculate_position(new_position)
        ticks_per_step = self.calculate_speed(self._current_position, new_position, time)
        move_data: bytearray = bytearray()
        move_data += new_position_steps.to_bytes(2, "big")
        move_data += ticks_per_step.to_bytes(3, "big")
        self._current_position = new_position
        return move_data