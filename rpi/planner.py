from __future__ import annotations

import time
from enum import IntEnum
from typing import Optional

import numpy as np

from controller import Controller
from kinematics import Move
from kinematics.base_kinematics import BaseKinematics
from endpoints import BaseJogEndpoint, program_endpoints
from parsers import ParserSettings, Program


class Planner:
    class MoveMode(IntEnum):
        AUTO = 0
        MANUAL = 1

    # Interpolate the move per step of X mm or degrees
    interpolation_step_length: float = 0.1

    def __init__(self, controller: Controller):
        self.controller: Controller = controller
        self.kinematic: BaseKinematics = controller.kinematic_settings.get_kinematics()
        self.jog_controller: BaseJogEndpoint = controller.move_settings.get_jog_controller(self.kinematic.inverse_kinematics_coordinate_names)

        ParserSettings().set_coordinate_names(self.kinematic.parse_coordinate_names)

        self.move_mode_changed = False
        self.move_mode: Planner.MoveMode = Planner.MoveMode.AUTO
        self.program: Optional[Program] = None

        self._cur_speed = 0
        self._cur_direction = np.array([0, 0, 0, 0, 0, 0])

        for endpoint in program_endpoints:
            endpoint.start()

    @classmethod
    def from_config(cls, config_file):
        return cls(Controller.from_config(config_file))

    def set_move_mode(self, move_mode: Planner.MoveMode):
        self.move_mode = move_mode
        if move_mode == Planner.MoveMode.AUTO:
            self.controller.set_move_mode(Controller.MoveMode.CACHED)
        elif move_mode == Planner.MoveMode.MANUAL:
            self.controller.set_move_mode(Controller.MoveMode.REALTIME)
        self.move_mode_changed = True

    def load_program(self):
        self.program = Program.program_buffer

    def run(self):
        while True:
            if self.move_mode == Planner.MoveMode.AUTO:
                while not self.move_mode_changed:
                    self.plan_program()
            elif self.move_mode == Planner.MoveMode.MANUAL:
                while not self.move_mode_changed:
                    direction = self.jog_controller.get_move()
                    self.jog(direction)
            self.move_mode_changed = False

    def jog(self, direction: list[float | int]):
        time_to_move = 0.05

        end_coordinates = self.kinematic.coordinates
        new_speed_per_axis = self.kinematic.calc_new_jog_velocity(direction, time_to_move)
        for i in range(len(end_coordinates)):
            end_coordinates[i] += new_speed_per_axis[i] * time_to_move

        end_joints = self.kinematic.inverse_kinematics(end_coordinates)
        if np.isnan(end_joints).any():
            return
        self.controller.move_steppers(np.degrees(end_joints), time_to_move)

    def run_move(self, move: Move):
        self.kinematic.create_velocity_profile(move)
        success, cur_speed = self.plan_move(move)
        print(success, cur_speed)

    def plan_program(self):
        if self.program is None:
            time.sleep(0.5)
            return
        self.program.make_absolute(self.kinematic.coordinates)
        moves = Move.from_program(self.program)
        self.kinematic.create_velocity_profile(moves)

        cur_speed = 0
        for move in moves:
            move.velocity.calc_based_on_start_velocity(cur_speed)
            succes, cur_speed = self.plan_move(move)
            if not succes:
                break

        self.program = None


    def plan_move(self, move: Move):
        interpolation_steps = max(int(move.velocity.distance / self.interpolation_step_length), 1)

        cur_speed = move.velocity.start_velocity

        start_coordinates = self.kinematic.coordinates
        end_coordinates = move.coordinate

        cur_coordinates = np.array(start_coordinates).astype(float)
        distance_traveled = self.interpolation_step_length
        move_per_step = ((np.array(end_coordinates) - np.array(start_coordinates)) / interpolation_steps).astype(float)

        for i in range(interpolation_steps):
            if self.move_mode_changed:
                return False, 0
            # Check for acceleration or deceleration
            if (move.velocity.distance - distance_traveled < move.velocity.braking_distance and
                    cur_speed >= move.velocity.end_velocity):
                distance_breaking = move.velocity.braking_distance - (move.velocity.distance - distance_traveled)
                cur_speed = np.sqrt(
                    move.velocity.cruise_velocity ** 2 - 2 * distance_breaking * move.velocity.acceleration
                )
                cur_speed = max(move.velocity.end_velocity, cur_speed)
            else:
                cur_speed = np.sqrt(
                    2 * distance_traveled * move.velocity.acceleration
                )
                cur_speed = min(move.velocity.cruise_velocity, cur_speed)

            time_to_move = self.interpolation_step_length / cur_speed

            # Calculate the next coordinates
            cur_coordinates += move_per_step
            cur_joints = self.kinematic.inverse_kinematics(tuple(cur_coordinates))
            if np.isnan(cur_joints).any():
                return False, 0
            # Check moves with the steppers maximum speed/acceleration
            stepper_longest_move = -1
            eta = time_to_move
            for j in range(len(self.controller.steppers)):
                if not self.controller.steppers[j].check_move(np.degrees(cur_joints[j]), time_to_move):
                    stepper_longest_move = j
                    stepper_eta = self.controller.steppers[j].ETA(np.degrees(cur_joints[j]))
                    eta = max(eta, stepper_eta)
                    cur_speed = max(0, self.interpolation_step_length / eta)
            # Move the steppers and kinematics
            if stepper_longest_move >= 0:
                self.controller.move_steppers_interpolated(np.degrees(cur_joints), eta)
            else:
                self.controller.move_steppers(np.degrees(cur_joints), time_to_move)
            self.kinematic.coordinates = cur_coordinates
            distance_traveled += self.interpolation_step_length
        return True, cur_speed

if __name__ == "__main__":
    controller = Controller.from_config("moveo.yaml")
    # controller.connect()
    # controller.reset()
    # controller.send_config()

    planner = Planner(controller)

    # planner.plan_move((0, 0, 600, np.radians(0), np.radians(0), np.radians(0)))
    # print()
    # planner.plan_move((300, 0, 525, np.radians(90), np.radians(0),  np.radians(0)))
    # print()
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(0)))
    # print()
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(25)))

