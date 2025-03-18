from typing import List

from pydantic import BaseModel, field_validator

from controller.pin import pin_types, validate_pin_type


class ManualPin(BaseModel):
    pin: pin_types
    static: bool = True
    value: bool = True

    @field_validator("pin", mode="before")
    @classmethod
    def _validate_pins(cls, value):
        return validate_pin_type(value)

    def get_config(self) -> List[bytearray]:
        pin_config = bytearray()
        pin_config += b'\x27'
        pin_config += b'\x00'
        pin_config += self.pin.pin_number_config
        if not self.static:
            return [pin_config]
        pin_set_output = bytearray()
        pin_set_output += b'\x15'
        if self.value:
            pin_set_output += b'\x01'
        else:
            pin_set_output += b'\x00'
        pin_set_output += self.pin.pin_number_config
        return [pin_config, pin_set_output]

