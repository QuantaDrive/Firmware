from __future__ import annotations

import os

from pydantic import BaseModel, Field, field_validator

from controller.pin import Direction

class Gpio(BaseModel):
    number: int = Field(default=31, ge=0, le=31)
    direction: Direction = Field(default=Direction.INPUT)
    inverted: bool = Field(default=False)

    @field_validator("number", mode="after")
    @classmethod
    def _validate_number(cls, value: int):
        if value in [0, 1] and os.environ.get("MCU_DEBUG") is not None:
            raise ValueError("GPIO 0 and 1 are reserved for debug probe")
        if value in [2, 3]:
            raise ValueError("GPIO 2 and 3 are reserved for serial to the shift registers")
        return value

    @field_validator("direction", mode="before")
    @classmethod
    def _validate_direction(cls, value):
        if isinstance(value, str):
            return Direction[value.upper()]
        return value

    @property
    def pin_number_config(self) -> bytes:
        return int((self.inverted << 5) + self.number).to_bytes(1, "big")
