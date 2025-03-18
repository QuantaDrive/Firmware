from abc import abstractmethod, ABC
from functools import cached_property
from typing import Literal, Tuple, Dict

from pydantic import BaseModel, Field

from controller.pin import Direction
from controller.pin.gpio import Gpio

multiplexers_lookup: Dict[str, Tuple[int, Direction]] = {}
shift_registers_lookup: Dict[str, int] = {}

def set_lookup_tables(multiplexers: Dict[str, Tuple[int, Direction]] = None, shift_registers: Dict[str, int] = None):
    global multiplexers_lookup, shift_registers_lookup
    if multiplexers:
        multiplexers_lookup = multiplexers
    if shift_registers:
        shift_registers_lookup = shift_registers

def validate_pin_type(value: str | int) -> dict:
    if isinstance(value, int):
        return {"type": "GPIO", "number": value}
    invert = False
    if value.startswith("~"):
        invert = True
        value = value[1:]
    if ':' not in value:
        return {"type": "GPIO", "number": int(value), "inverted": invert}
    name, address = value.split(':')
    if name in multiplexers_lookup:
        return {"type": "Multiplexer", "mux_id": multiplexers_lookup[name][0], "mux_address": int(address),
                "inverted": invert, "direction": multiplexers_lookup[name][1]}
    elif name in shift_registers_lookup:
        return {"type": "Shift-Register", "sr_id": shift_registers_lookup[name], "sr_address": int(address), "inverted": invert}
    else:
        raise ValueError(f"Unknown pin type {name}")

class Pin(BaseModel):
    type: Literal[""]

    @property
    @abstractmethod
    def pin_number_config(self) -> bytes:
        pass

class GpioPin(Gpio, Pin, ABC):
    type: Literal["GPIO"]

    @property
    def pin_number_config(self) -> bytes:
        return super().pin_number_config

class MultiplexerPin(Pin, ABC):
    type: Literal["Multiplexer"]
    direction: Direction = Field(default=Direction.INPUT)
    inverted: bool = Field(default=False)
    mux_id: int = Field(ge=0, le=3)
    mux_address: int = Field(ge=0, le=7)

    @property
    def pin_number_config(self) -> bytes:
        return int((1 << 6) + (self.inverted << 5) + (self.mux_id << 3) + self.mux_address).to_bytes(1, "big")


class ShiftRegisterPin(Pin, ABC):
    type: Literal["Shift-Register"]
    inverted: bool = Field(default=False)
    sr_id: int = Field(ge=0, le=3)
    sr_address: int = Field(ge=0, le=7)

    @cached_property
    def direction(self):
        return Direction.OUTPUT

    @property
    def pin_number_config(self) -> bytes:
        return int((1 << 7) + (self.inverted << 5) + (self.sr_id << 3) + self.sr_address).to_bytes(1, "big")