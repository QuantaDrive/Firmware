from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Literal

import numpy.typing as npt
import sympy as sp
from pydantic import BaseModel, Field


class BaseKinematics(ABC):
    # These need to be all lowercase
    forward_kinematics_joint_names: tuple[str] = ()
    inverse_kinematics_coordinate_names:  tuple[str] = ()
    parse_coordinate_names: tuple[str] = ()

    def __init__(self, settings: type[BaseKinematicsModel]):
        self._cur_coordinates: list[float | int] = []
        self._cur_direction: npt.NDArray(float | int) = []
        self._cur_speed: npt.NDArray(float | int) = []

        self.settings = settings

    @property
    def coordinates(self) -> list[float | int]:
        if len(self._cur_coordinates) == 0:
            joint_values = tuple(0 for _ in range(len(self.forward_kinematics_joint_names)))
            self._cur_coordinates = list(self.forward_kinematics(joint_values))
        return self._cur_coordinates

    @coordinates.setter
    def coordinates(self, coordinates: list[float | int]):
        self._cur_coordinates = coordinates

    @staticmethod
    @abstractmethod
    def get_length(start_coordinates: list[float | int], end_coordinates: list[float | int]):
        pass

    @staticmethod
    @abstractmethod
    def convert_coordinates(coordinates: list[float | int | None]) -> list[float | int | None]:
        pass

    @abstractmethod
    def calc_new_jog_speed(self, new_direction: list[float | int], time_to_move: float | int):
        pass

    @abstractmethod
    def forward_kinematics(self, joints: tuple[float | int, ...], toolframe_matrix=sp.eye(4)) -> tuple[float | int, ...]:
        pass

    @abstractmethod
    def inverse_kinematics(self, coordinates: tuple[float | int, ...], toolframe_matrix=sp.eye(4)) -> tuple[float | int, ...]:
        pass

class BaseKinematicsModel(BaseModel):
    type: Literal[""]

    jog_velocity: float = Field(default=1)  # mm per second
    jog_decel: float = Field(default=5)  # mm per second
    max_velocity: float = Field(default=25)  # mm per second
    max_accel: float = Field(default=2.5)  # mm per second^2

    def get_kinematics(self):
        pass