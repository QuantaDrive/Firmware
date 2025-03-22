from typing import Literal

from pydantic import BaseModel

class MoveSettings(BaseModel):
    jog_controller: Literal[""]
    jog_controller_mapping: list[dict[str, str]]

    def get_jog_controller(self, inverse_kinematics_coordinate_names: tuple[str]):
        pass