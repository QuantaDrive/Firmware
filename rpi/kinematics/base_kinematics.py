from abc import ABC, abstractmethod
from typing import Tuple

import sympy as sp


class BaseKinematics(ABC):
    forward_kinematics_joint_names: Tuple[str] = ()
    inverse_kinematics_coordinate_names:  Tuple[str] = ()

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