from pydantic import BaseModel, field_validator

from controller.pin import pin_types, validate_pin_type


class InterruptPin(BaseModel):
    pin: pin_types

    @field_validator("pin", mode="before")
    @classmethod
    def _validate_pins(cls, value):
        return validate_pin_type(value)

    def get_config(self) -> bytearray:
        pin_config = bytearray()
        pin_config += b'\x28'
        pin_config += b'\x00'
        pin_config += self.pin.pin_number_config
        return pin_config