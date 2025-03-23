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
            self.acceleration: float = 0
            self.start_velocity: float = 0
            self.cruise_velocity: float = 0
            self.end_velocity: float = 0
            self.braking_distance: float = 0

        def calc_max_velocity(self):
            return np.sqrt(
                self.acceleration * self.distance + (self.start_velocity ** 2 + self.end_velocity ** 2) / 2
            )

        def calc_braking_distance(self):
            self.braking_distance = (self.cruise_velocity ** 2 - self.end_velocity ** 2) / (2 * self.acceleration)

        def calc_based_on_start_velocity(self, start_velocity: float):
            if start_velocity > self.start_velocity:
                return
            self.start_velocity = start_velocity
            max_velocity = self.calc_max_velocity()
            if max_velocity < self.cruise_velocity:
                self.cruise_velocity = max_velocity
                if self.cruise_velocity < self.end_velocity:
                    self.end_velocity = self.cruise_velocity
            self.calc_braking_distance()

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
            move.velocity.cruise_velocity = line.speed
            moves.append(move)
        return moves
