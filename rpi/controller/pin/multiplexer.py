from typing import List

from pydantic import BaseModel, Field, field_validator

from controller.pin.gpio import Gpio
from controller.pin import Direction, validate_pin_type


class Multiplexer(BaseModel):
    id: int = Field(default=0, ge=0, le=3)
    name: str
    direction: Direction = Field(default=Direction.INPUT)
    data_pin: Gpio
    enable_pin: Gpio
    address_pins: List[Gpio] = Field(..., alias="addr_pins", min_length=3, max_length=3)

    @field_validator("direction", mode="before")
    @classmethod
    def _validate_direction(cls, value):
        if isinstance(value, str):
            return Direction[value.upper()]
        return value

    @field_validator("data_pin", "enable_pin", mode="before")
    @classmethod
    def _validate_pin(cls, value: str | int):
        return validate_pin_type(value)

    @field_validator("address_pins", mode="before")
    @classmethod
    def _validate_pins(cls, value: List):
        pins = []
        for pin in value:
            pins.append(validate_pin_type(pin))
        return pins

    def get_config(self) -> bytearray:
        command = bytearray()
        command += b'\x24'
        if self.direction == Direction.OUTPUT:
            command += (self.id + 4).to_bytes(1, "big")
        else:
            command += self.id.to_bytes(1, "big")
        command += self.data_pin.to_bytes(1, "big")
        command += self.enable_pin.to_bytes(1, "big")
        for pin in self.address_pins:
            command += pin.to_bytes(1, "big")
        return command