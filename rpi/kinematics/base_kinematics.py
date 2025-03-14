from abc import ABC, abstractmethod
from typing import Tuple, Literal

import sympy as sp
from pydantic import BaseModel


class BaseKinematics(ABC):
    # These need to be all lowercase
    forward_kinematics_joint_names: Tuple[str] = ()
    inverse_kinematics_coordinate_names:  Tuple[str] = ()

    _cur_coordinates: Tuple[float | int] = ()

    @property
    def coordinates(self):
        if len(self._cur_coordinates) == 0:
            joint_values = tuple(0 for _ in range(len(self.forward_kinematics_joint_names)))
            self._cur_coordinates = self.forward_kinematics(joint_values)
        return self._cur_coordinates

    @coordinates.setter
    def coordinates(self, coordinates):
        self._cur_coordinates = coordinates

    @staticmethod
    @abstractmethod
    def get_length(start_coordinates: Tuple[float | int], end_coordinates: Tuple[float | int]):
        pass

    @abstractmethod
    def forward_kinematics(self, joints: Tuple[float | int], toolframe_matrix=sp.eye(4)):
        pass

    @abstractmethod
    def inverse_kinematics(self, coordinates: Tuple[float | int], toolframe_matrix=sp.eye(4)):
        pass

class BaseKinematicsModel(BaseModel):
    type: Literal[""]

    def get_kinematics(self):
        pass