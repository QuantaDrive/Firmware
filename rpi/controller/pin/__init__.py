from typing import Annotated, Union

from pydantic import Field

from .direction import Direction
from .pin import validate_pin_type, Pin, GpioPin, MultiplexerPin, ShiftRegisterPin
from .pin import set_lookup_tables as pin_set_lookup_tables

pin_types = Annotated[
    Union[GpioPin, MultiplexerPin, ShiftRegisterPin],
    Field(discriminator="type")
]

from .multiplexer import Multiplexer
from .shift_register import ShiftRegister

from .manual_pin import ManualPin
from .interrupt_pin import InterruptPin

