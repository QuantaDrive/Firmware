from __future__ import annotations

import numpy as np
import numpy.typing as npt

from parsers import Program


class Move:
    class Velocity:
        def __init__(self):
            """
            if cruise_velocity == end_velocity:
                braking_distance = 0
                the move is not long enough to get to cruise_velocity
            if braking_distance == 0:
                the move is not long enough to get to end_velocity
            """
            self.id: int = 0
            self.distance: float = 0
            self.acceleration: npt.NDArray = np.array([])
            self.deceleration: npt.NDArray = np.array([])
            self.cur_start_velocity: float = 0
            self.cruise_velocity: npt.NDArray = np.array([])
            self.cur_end_velocity: float = 0
            self.braking_distance: float = 0

        @property
        def cur_acceleration(self) -> float:
            return float(self.acceleration[self.id])

        @cur_acceleration.setter
        def cur_acceleration(self, value: float):
            self.acceleration[self.id] = value

        @property
        def cur_deceleration(self) -> float:
            return float(self.deceleration[self.id])

        @cur_deceleration.setter
        def cur_deceleration(self, value: float):
            self.deceleration[self.id] = value

        @property
        def cur_cruise_velocity(self) -> float:
            return float(self.cruise_velocity[self.id])

        @cur_cruise_velocity.setter
        def cur_cruise_velocity(self, value: float):
            self.cruise_velocity[self.id] = value

        def calc_max_velocity(self, length: npt.NDArray[float]):
            return np.sqrt(
                self.cur_acceleration * length[self.id] + (self.cur_start_velocity ** 2 + self.cur_end_velocity ** 2) / 2
            )

        def calc_braking_distance(self):
            self.braking_distance = (self.cur_cruise_velocity ** 2 - self.cur_end_velocity ** 2) / (2 * self.cur_acceleration)

        def calc_secondary_moves(self, length: npt.NDArray[float]):
            t_a = (self.cur_cruise_velocity - self.cur_start_velocity) / self.cur_acceleration
            t_d = (self.cur_cruise_velocity - self.cur_end_velocity) / self.cur_deceleration
            s_c = length[self.id] - (self.cur_cruise_velocity + self.cur_start_velocity) * t_a / 2 - self.braking_distance
            t_c = s_c / self.cur_cruise_velocity
            for i in range(len(length)):
                if i == self.id:
                    continue
                if t_a == 0 or t_d == 0:
                    t = t_a + t_c + t_d
                    cruise_velocity = length[i] / t
                    acceleration = cruise_velocity / (t / 2)
                    self.cruise_velocity[i] = cruise_velocity
                    self.acceleration[i] = acceleration
                    self.deceleration[i] = acceleration
                else:
                    cruise_velocity = length[i] / (0.5*t_a + t_c + 0.5*t_d)
                    acceleration = cruise_velocity / t_a
                    deceleration = cruise_velocity / t_d
                    self.cruise_velocity[i] = cruise_velocity
                    self.acceleration[i] = acceleration
                    self.deceleration[i] = deceleration


    def __init__(self):
        self.velocity = Move.Velocity()
        self.coordinate: npt.NDArray[float | int] = np.array([])

    @staticmethod
    def from_program(program: Program) -> list[Move]:
        moves = []
        for line in program.lines:
            if line.relative:
                raise ValueError("You must make the program absolute first")
            move = Move()
            move.coordinate = line.coordinates
            move.velocity.cruise_velocity = np.array(line.speed)
            moves.append(move)
        return moves
