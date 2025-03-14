from typing import Literal, List, Dict

from pydantic import BaseModel, Field

class MoveSettings(BaseModel):
    jog_controller: Literal[""]
    jog_controller_mapping: List[Dict[str, str]]

    jog_velocity: float = Field(default=1)       # mm per second
    max_velocity: float = Field(default=25)      # mm per second
    max_accel: float = Field(default=2.5)        # mm per second^2

    def get_jog_controller(self):
        pass