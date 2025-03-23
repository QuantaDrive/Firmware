from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Literal

import numpy as np
import numpy.typing as npt
import sympy as sp
from pydantic import BaseModel, Field

from kinematics import Move


class BaseKinematics(ABC):
    # These need to be all lowercase
    forward_kinematics_joint_names: tuple[str] = ()
    inverse_kinematics_coordinate_names:  tuple[str] = ()
    parse_coordinate_names: tuple[str] = ()

    def __init__(self, settings: type[BaseKinematicsModel]):
        self._cur_coordinates: npt.NDArray[float | int] = np.array([])
        self._cur_direction: npt.NDArray[float | int]
        self._cur_speed: npt.NDArray[float | int]

        self.settings = settings

    @property
    def coordinates(self) -> npt.NDArray[float | int]:
        if len(self._cur_coordinates) == 0:
            joint_values = tuple(0 for _ in range(len(self.forward_kinematics_joint_names)))
            self._cur_coordinates = np.array(self.forward_kinematics(joint_values))
        return self._cur_coordinates

    @coordinates.setter
    def coordinates(self, coordinates: list[float | int] | npt.NDArray[float | int]):
        coordinates = np.array(coordinates)
        self._cur_coordinates = coordinates

    @abstractmethod
    def convert_coordinates(self, coordinates: list[float | int | None]) -> npt.NDArray[float | int]:
        pass

    @staticmethod
    @abstractmethod
    def normalize_direction(direction: list[float | int] | npt.NDArray[float | int]):
        pass

    @staticmethod
    @abstractmethod
    def angle_between_directions(direction1: npt.NDArray[float | int], direction2: npt.NDArray[float | int]) -> npt.NDArray[float | int]:
        pass

    @staticmethod
    @abstractmethod
    def get_length(start_coordinates: npt.NDArray[float | int], end_coordinates: npt.NDArray[float | int]) -> tuple[npt.NDArray[float | int], int]:
        pass

    @abstractmethod
    def get_speed_dir_size(self, speed: list[float | int] = None) -> npt.NDArray[float | int]:
        pass

    @abstractmethod
    def calc_new_jog_velocity(self, new_direction: list[float | int], time_to_move: float | int):
        pass

    def _set_start_velocity(self, moves: list[Move], i: int, start_velocity: npt.NDArray[float]):
        if i == 0:
            return

        if i == 1:
            pre_lengths, pre_velocity_id = self.get_length(self._cur_coordinates, moves[i].coordinate)
        else:
            pre_lengths, pre_velocity_id = self.get_length(moves[i - 2].coordinate, moves[i - 1].coordinate)

        moves[i].velocity.start_velocity = start_velocity
        moves[i - 1].velocity.end_velocity = start_velocity

        pre_max_speed = moves[i - 1].velocity.calc_max_velocity(pre_lengths)
        if pre_max_speed < moves[i - 1].velocity.cur_cruise_velocity:
            moves[i - 1].velocity.cur_cruise_velocity = pre_max_speed

        moves[i - 1].velocity.calc_braking_distance()

        if pre_lengths[pre_velocity_id] < moves[i - 1].velocity.braking_distance:
            # start speed to high for end speed, deceleration and distance
            # Move only decelerates but cant get to end velocity
            # So move the start velocity down so it can get to the end velocity
            pre_start_velocity = np.sqrt(
                2 * pre_lengths[pre_velocity_id] * moves[i - 1].velocity.cur_deceleration +
                moves[i - 1].velocity.cur_end_velocity ** 2
            )
            self._set_start_velocity(moves, i - 1, pre_start_velocity)

        moves[i - 1].velocity.calc_secondary_moves(pre_lengths)

    def create_velocity_profile(self, moves: list[Move]):
        moves[0].coordinate = self.convert_coordinates(moves[0].coordinate)

        lengths, velocity_id = self.get_length(self._cur_coordinates, moves[0].coordinate)
        pre_lengths, pre_velocity_id = lengths, velocity_id
        for i in range(len(moves)):
            if i > 0:
                moves[i].coordinate = self.convert_coordinates(moves[i].coordinate)
                lengths, velocity_id = self.get_length(moves[i - 1].coordinate, moves[i].coordinate)

            moves[i].velocity.id = velocity_id
            moves[i].velocity.distance = lengths[i]

            cruise_velocity = np.array([0, 0, 0, 0])
            cruise_velocity[velocity_id] = moves[i].velocity.cruise_velocity[0]
            moves[i].velocity.cruise_velocity = cruise_velocity

            acceleration = np.array([0, 0, 0, 0])
            acceleration[velocity_id] = self.settings.max_accel
            moves[i].velocity.acceleration = acceleration
            moves[i].velocity.deceleration = acceleration

            moves[i].velocity.calc_secondary_moves(lengths)

            if i == 0:
                continue

            if velocity_id == pre_velocity_id:
                cur_direction = moves[i].coordinate - moves[i - 1].coordinate
                pre_direction = moves[i - 1].coordinate - moves[i - 2].coordinate
                angle_between_directions = self.angle_between_directions(cur_direction, pre_direction)

                moves[i - 1].velocity.end_velocity = moves[i].velocity.cruise_velocity * angle_between_directions[
                    velocity_id]

                pre_max_speed = moves[i - 1].velocity.calc_max_velocity(pre_lengths)
                if pre_max_speed < moves[i - 1].velocity.cur_cruise_velocity:
                    moves[i - 1].velocity.cur_cruise_velocity = pre_max_speed

                moves[i - 1].velocity.calc_braking_distance()

                pre_accelerating: bool = moves[i - 1].velocity.cur_start_velocity < moves[
                    i - 1].velocity.cur_end_velocity
                if pre_accelerating and pre_max_speed < moves[i - 1].velocity.cur_end_velocity:
                    # end speed to high for start speed, acceleration and distance
                    # Move only accelerates but cant get to end velocity
                    # So move the end velocity to the highest velocity possible
                    moves[i].velocity.cur_end_velocity = pre_max_speed
                elif not pre_accelerating and pre_lengths[pre_velocity_id] < moves[i - 1].velocity.braking_distance:
                    # start speed to high for end speed, deceleration and distance
                    # Move only decelerates but cant get to end velocity
                    # So move the start velocity down so it can get to the end velocity
                    pre_start_velocity = np.sqrt(
                        2 * pre_lengths[pre_velocity_id] * moves[i - 1].velocity.cur_deceleration +
                        moves[i - 1].velocity.cur_end_velocity ** 2
                    )
                    self._set_start_velocity(moves, i - 1, pre_start_velocity)

                moves[i - 1].velocity.calc_secondary_moves(pre_lengths)

            moves[i].velocity.start_velocity = moves[i - 1].velocity.end_velocity
            pre_lengths, pre_velocity_id = lengths, velocity_id

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