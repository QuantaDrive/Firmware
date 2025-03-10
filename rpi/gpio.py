from __future__ import annotations

import os
from enum import IntEnum
from typing import Any

from pydantic import BaseModel, Field, model_validator, field_validator


class Direction(IntEnum):
    INPUT = 0
    OUTPUT = 1

class Gpio(BaseModel):
    number: int = Field(default=31)
    direction: Direction = Field(default=Direction.INPUT)
    inverted: bool = Field(default=False)

    @field_validator("number", mode="after")
    @classmethod
    def _validate_number(cls, value: int):
        if value < 0 or value > 31:
            raise ValueError("Number must be between 0 and 31")
        if value in [0, 1] and os.environ.get("MCU_DEBUG") is not None:
            raise ValueError("GPIO 0 and 1 are reserved for debug probe")
        return value

    @field_validator("direction", mode="before")
    @classmethod
    def _validate_direction(cls, value):
        if isinstance(value, str):
            return Direction[value.upper()]
        return value

    @model_validator(mode="before")
    @classmethod
    def _validate_number_prefix(cls, data: Any) -> Any:
        if isinstance(data, dict):
            return data
        if isinstance(data, int):
            return {"number": data}
        if isinstance(data, str):
            if data.startswith("~"):
                return {"number": int(data[1:]), "inverted": True}
            return {"number": int(data)}

    @property
    def pin_number_config(self) -> bytes:
        return int(self.number + 128 * self.inverted).to_bytes(1, "big")
