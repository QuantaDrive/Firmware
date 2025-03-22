from __future__ import annotations

import time
from enum import IntEnum
from typing import Optional

import numpy as np

from controller import Controller
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
                    if self.program is None:
                        time.sleep(0.1)
                        continue
                    for line in self.program.lines:
                        coordinates = self.kinematic.convert_coordinates(line.coordinates)
                        self.plan_move(coordinates, line.speed, line.relative)
                    self.program = None
            elif self.move_mode == Planner.MoveMode.MANUAL:
                while not self.move_mode_changed:
                    direction = self.jog_controller.get_move()
                    self.jog(direction)
            self.move_mode_changed = False

    def jog(self, direction: list[float | int]):
        time_to_move = 0.05

        end_coordinates = self.kinematic.coordinates
        new_speed_per_axis = self.kinematic.calc_new_jog_speed(direction, time_to_move)
        for i in range(len(end_coordinates)):
            end_coordinates[i] += new_speed_per_axis[i] * time_to_move

        end_joints = self.kinematic.inverse_kinematics(tuple(end_coordinates))
        if np.isnan(end_joints).any():
            return
        self.controller.move_steppers(end_joints, time_to_move)


    def plan_move(self, coordinates: list[float | int | None], speed: float = 0,
                  relative: bool = False):
        acceleration = self.controller.move_settings.max_accel
        if speed == 0:
            speed = self.controller.move_settings.max_velocity

        coordinates = tuple(
            self.kinematic.coordinates[i] if coord is None else coord
            for i, coord in enumerate(coordinates)
        )

        start_coordinates = self.kinematic.coordinates
        end_coordinates = coordinates
        if relative:
            end_coordinates += start_coordinates

        move_length = self.kinematic.get_length(start_coordinates, end_coordinates)
        if move_length == 0:
            return False
        interpolation_steps = max(int(move_length / self.interpolation_step_length), 1)

        # Take the starting speed same as the interpolation step length * 2
        # So the first move is 0.2 second long
        min_speed = self.interpolation_step_length * 2
        cur_speed = min_speed

        time_to_move = self.interpolation_step_length / cur_speed

        cur_coordinates = np.array(start_coordinates).astype(float)
        move_length_left = move_length
        move_per_step = ((np.array(end_coordinates) - np.array(start_coordinates)) / interpolation_steps).astype(float)

        accelerating = True
        braking_distance = 0
        for i in range(interpolation_steps):
            if self.move_mode_changed:
                return False
            # Calculate the next coordinates
            cur_coordinates += move_per_step
            cur_joints = self.kinematic.inverse_kinematics(tuple(cur_coordinates))
            if np.isnan(cur_joints).any():
                return False
            # Check moves with the steppers maximum speed/acceleration
            stepper_longest_move = -1
            ETA = time_to_move
            for j in range(len(self.controller.steppers)):
                if not self.controller.steppers[j].check_move(np.degrees(cur_joints[j]), time_to_move):
                    stepper_longest_move = j
                    stepper_ETA = self.controller.steppers[j].ETA(np.degrees(cur_joints[j]))
                    ETA = max(ETA, stepper_ETA)
                    cur_speed = max(min_speed, self.interpolation_step_length / ETA)
            # Move the steppers and kinematics
            if stepper_longest_move >= 0:
                self.controller.move_steppers_interpolated(np.degrees(cur_joints), ETA)
            else:
                self.controller.move_steppers(np.degrees(cur_joints), time_to_move)
            self.kinematic.coordinates = cur_coordinates
            # Check for acceleration or deceleration
            if acceleration == 0:
                continue
            move_length_left -= self.interpolation_step_length
            if move_length_left < move_length / 2:
                braking_distance = np.abs(((self.interpolation_step_length * 2) ** 2 - cur_speed ** 2) / (2 * acceleration))
            if move_length_left < braking_distance:
                accelerating = False
            if accelerating:
                cur_speed += acceleration * time_to_move
                cur_speed = min(speed, cur_speed)
            else:
                cur_speed -= acceleration * time_to_move
                cur_speed = max(min_speed, cur_speed)
            time_to_move = self.interpolation_step_length / cur_speed
        return True

if __name__ == "__main__":
    controller = Controller.from_config("moveo.yaml")
    # controller.connect()
    # controller.reset()
    # controller.send_config()

    planner = Planner(controller)

    planner.plan_move((0, 0, 600, np.radians(0), np.radians(0), np.radians(0)))
    print()
    # planner.plan_move((300, 0, 525, np.radians(90), np.radians(0),  np.radians(0)))
    # print()
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(0)))
    # print()
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(25)))

