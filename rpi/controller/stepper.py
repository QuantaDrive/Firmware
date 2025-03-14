from enum import IntEnum
from functools import cached_property
from math import floor
from typing import Optional, List

import numpy as np
from pydantic import BaseModel, Field, PositiveFloat, field_validator, PositiveInt, computed_field, PrivateAttr

from controller import Gpio

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
        value.direction = Gpio.Direction.OUTPUT
        return value

    @field_validator("diag_fault_pin", mode="after")
    @classmethod
    def _validate_input_pins(cls, value: Gpio):
        value.direction = Gpio.Direction.INPUT
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
    microsteps: PositiveInt = Field(default=1)
    steps_per_rev: PositiveInt = Field(default=200)
    min_position: float = Field(default=0)
    max_position: float = Field(default=np.inf)
    max_velocity: float = Field(default=np.inf)
    max_accel: float = Field(default=np.inf)

    _current_position: float = PrivateAttr(default=0)
    _cur_speed: float = PrivateAttr(default=0)  # rotations per second
    _time_skipped: float = PrivateAttr(default=0)

    def __init__(self, **data):
        super().__init__(**data)

    @computed_field(return_type=int)
    def _current_position_steps(self):
        return self.get_steps(self._current_position)

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

    def check_move(self, new_position: float, time: float, relative: bool = False):
        if relative:
            delta_position = new_position
        else:
            delta_position = new_position - self._current_position
        if delta_position < 10:
            return True
        speed = np.abs(delta_position / (time + self._time_skipped))
        acceleration = (speed - self._cur_speed) / (time + self._time_skipped)
        if speed > self.max_velocity or acceleration > self.max_accel:
            return False
        return True

    def ETA(self, new_position: float, relative: bool = False):
        if relative:
            delta_position = new_position
        else:
            delta_position = new_position - self._current_position
        delta_position = np.abs(delta_position)

        position_to_full_speed = np.abs((self.max_velocity ** 2 - self._cur_speed ** 2) / (2 * self.max_accel))
        if delta_position < position_to_full_speed:
            time_to_full_speed = (- self._cur_speed + np.sqrt(self._cur_speed ** 2 + 2 * self.max_accel * delta_position)) / (2 * self.max_accel)
            return time_to_full_speed
        time_to_full_speed = (self.max_velocity - self._cur_speed) / self.max_accel
        remaining_position = delta_position - position_to_full_speed
        remaining_time = remaining_position / self.max_velocity
        return time_to_full_speed + remaining_time

    def interpolate_position(self, new_position: float, time: float, time_to_move: float, relative: bool = False):
        """
        :param new_position: Position to get to
        :param time: Total time to get to the position
        :param time_to_move: Time 1 move will take
        :param relative:
        :return:
        """
        if relative:
            delta_position = new_position
        else:
            delta_position = new_position - self._current_position
        acceleration = 2 * (delta_position - self._cur_speed * time) / (time ** 2)
        interpolation_steps = floor(time / time_to_move)
        interpolation_steps_left = time / time_to_move - interpolation_steps
        current_position = self._current_position
        cur_speed = self._cur_speed
        for i in range(interpolation_steps):
            current_position += cur_speed * time_to_move + 0.5 * acceleration * time_to_move ** 2
            cur_speed += acceleration * time_to_move
            yield current_position
        time_to_move = time_to_move * interpolation_steps_left
        current_position += cur_speed * time_to_move + 0.5 * acceleration * time_to_move ** 2
        yield current_position

    def get_steps(self, position: float) -> int:
        return round((position - self.min_position) * self.final_steps_per_mm)

    def calculate_position(self, position: float) -> int:
        if position > self.max_position or position < self.min_position:
            print("Position out of range: " + str(position))
            position = np.clip(position, self.min_position, self.max_position)
            #raise ValueError("Position out of range")
        return self.get_steps(position)

    def calculate_speed(self, position_steps: int, time: float) -> int:
        delta_steps = np.abs(position_steps - self._current_position_steps)
        if delta_steps == 0:
            return 0
        steps_per_second = delta_steps / time
        ticks_per_step = 50000 / steps_per_second
        return int(max(ticks_per_step, 1))

    def move(self, new_position: float, time: float, relative: bool = False) -> bytearray:
        if relative:
            new_position += self._current_position
        new_position_steps = self.calculate_position(new_position)
        # time skipped is not included because the move needs to be done at this time otherwise this stepper will hold up the other steppers
        ticks_per_step = self.calculate_speed(new_position_steps, time)
        move_data: bytearray = bytearray()
        move_data += new_position_steps.to_bytes(2, "big")
        move_data += ticks_per_step.to_bytes(3, "big")
        if ticks_per_step != 0: # Because if the speed is 0, the position will not change
            self._cur_speed = np.abs((new_position - self._current_position) / (time + self._time_skipped))
            self._current_position = new_position
            self._time_skipped = 0
        elif self._current_position != new_position:
            self._time_skipped += time
        return move_data