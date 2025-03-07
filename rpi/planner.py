from typing import Type, Tuple

import numpy as np

from controller import Controller
from kinematics.base_kinematics import BaseKinematics


class Planner:
    # Interpolate the move per step of X mm or degrees
    interpolation_step_length: float = 0.5

    def __init__(self, controller: Controller):
        self.controller: Controller = controller
        self.kinematic: Type[BaseKinematics] = controller.kinematic_settings.get_kinematics()

    def plan_move(self, coordinates: Tuple[float | int], speed: float = 0, acceleration: float = 0):
        if speed == 0:
            speed = self.controller.max_velocity
        if acceleration == 0:
            acceleration = self.controller.max_accel

        start_coordinates = self.kinematic.coordinates
        end_coordinates = coordinates

        move_length = self.kinematic.get_length(start_coordinates, end_coordinates)
        interpolation_steps = int(move_length / self.interpolation_step_length)

        cur_speed = self.controller.start_velocity
        time_to_move = self.interpolation_step_length / cur_speed

        cur_coordinates = np.array(start_coordinates).astype(float)
        move_per_step = ((np.array(end_coordinates) - np.array(start_coordinates)) / interpolation_steps).astype(float)
        #DEBUG
        # cur_joints = self.kinematic.inverse_kinematics(tuple(cur_coordinates))
        # print(np.round(np.degrees(cur_joints), 2))
        for i in range(interpolation_steps):
            cur_coordinates += move_per_step
            # print(cur_coordinates)
            cur_joints = self.kinematic.inverse_kinematics(tuple(cur_coordinates))
            # print(np.round(np.degrees(cur_joints), 2))
            self.controller.move_steppers(np.degrees(cur_joints), time_to_move)
            self.kinematic.coordinates = cur_coordinates
            cur_speed += acceleration * time_to_move
            cur_speed = min(speed, cur_speed)
            time_to_move = self.interpolation_step_length / cur_speed


if __name__ == "__main__":
    from controller import Controller
    controller = Controller.from_config("moveo.yaml")
    # controller.connect()
    # controller.reset()
    # controller.send_config()

    planner = Planner(controller)
    planner.plan_move((301.5, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(0)))
    print()
    planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(0)))
    print()
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(15)))

