from __future__ import annotations

import time
from enum import IntEnum
from typing import Type, Tuple, List

import numpy as np

from controller import Controller
from kinematics.base_kinematics import BaseKinematics
from endpoints.base_jog_controller import BaseJogController

class Planner:
    class MoveMode(IntEnum):
        AUTO = 0
        MANUAL = 1

    class ProgramLine:
        def __init__(self, coordinates, speed, relative = False):
            self.coordinates = coordinates
            self.speed = speed
            self.relative = relative

    # Interpolate the move per step of X mm or degrees
    interpolation_step_length: float = 0.1

    def __init__(self, controller: Controller):
        self.move_mode_changed = False
        self.move_mode: Planner.MoveMode = Planner.MoveMode.AUTO
        self.controller: Controller = controller
        self.kinematic: Type[BaseKinematics] = controller.kinematic_settings.get_kinematics()
        self.jog_controller: Type[BaseJogController] = controller.move_settings.get_jog_controller()
        self.jog_controller.config_coordinate_names(self.kinematic.inverse_kinematics_coordinate_names)
        self.program: List[Planner.ProgramLine] = []

    @classmethod
    def from_config(cls, config_file):
        return cls(Controller.from_config(config_file))

    def set_move_mode(self, move_mode: Planner.MoveMode):
        self.move_mode = move_mode
        self.move_mode_changed = True

    def run(self):
        while True:
            if self.move_mode == Planner.MoveMode.AUTO:
                while not self.move_mode_changed:
                    for line in self.program:
                        self.plan_move(line.coordinates, line.speed, line.relative)
                    if len(self.program) == 0:
                        time.sleep(0.1)
            elif self.move_mode == Planner.MoveMode.MANUAL:
                while not self.move_mode_changed:
                    coordinates = self.jog_controller.get_move()
                    time_between_moves = self.plan_move(coordinates, jog=True)
                    time.sleep(max(0, time_between_moves - 0.5))
            self.move_mode_changed = False

    def plan_move(self, coordinates: Tuple[float | int], speed: float = 0,
                  relative: bool = False, jog: bool = False):
        acceleration = self.controller.move_settings.max_accel
        if speed == 0:
            speed = self.controller.move_settings.max_velocity

        start_coordinates = self.kinematic.coordinates
        end_coordinates = coordinates
        if relative:
            end_coordinates += start_coordinates

        move_length = self.kinematic.get_length(start_coordinates, end_coordinates)
        interpolation_steps = int(move_length / self.interpolation_step_length)

        # Take the starting speed same as the interpolation step length * 2
        # So the first move is 0.5 second long
        min_speed = self.interpolation_step_length * 2
        cur_speed = min_speed
        if jog:
            cur_speed = self.controller.jog_velocity
            acceleration = 0

        time_to_move = self.interpolation_step_length / cur_speed

        cur_coordinates = np.array(start_coordinates).astype(float)
        move_length_left = move_length
        move_per_step = ((np.array(end_coordinates) - np.array(start_coordinates)) / interpolation_steps).astype(float)

        accelerating = True
        braking_distance = 0
        total_time_moved = 0
        for i in range(interpolation_steps):
            # Calculate the next coordinates
            cur_coordinates += move_per_step
            cur_joints = self.kinematic.inverse_kinematics(tuple(cur_coordinates))
            # print("Joints: ", np.round(np.degrees(cur_joints), 2))
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
                total_time_moved += time_to_move
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
        return total_time_moved


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

