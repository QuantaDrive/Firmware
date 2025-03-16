from pydantic import BaseModel, Field, field_validator

from controller.pin import Direction, validate_pin_type
from controller.pin.gpio import Gpio

class ShiftRegister(BaseModel):
    id: int = Field(default=0, ge=0, le=3)
    name: str
    direction: Direction = Direction.OUTPUT
    latch_pin: Gpio
    offset: int = Field(default=0)  # Used when multiple shift registers are chained (0 = first, 1 = second, etc.)

    @field_validator("latch_pin", mode="before")
    @classmethod
    def _validate_pin(cls, value: str | int):
        return validate_pin_type(value)

    def get_config(self) -> bytearray:
        command = bytearray()
        command += b'\x25'
        command += (self.offset << 2 + self.id).to_bytes(1, "big")
        command += self.latch_pin.to_bytes(1, "big")
        return command
