from typing import Type, Tuple

import numpy as np

from controller import Controller
from kinematics.base_kinematics import BaseKinematics


class Planner:
    # Interpolate the move per step of X mm or degrees
    interpolation_step_length: float = 0.1

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

        # Take the starting speed same as the interpolation step length * 2
        # So the first move is 0.5 second long
        cur_speed = self.interpolation_step_length * 2
        time_to_move = self.interpolation_step_length / cur_speed

        cur_coordinates = np.array(start_coordinates).astype(float)
        move_length_left = move_length
        move_per_step = ((np.array(end_coordinates) - np.array(start_coordinates)) / interpolation_steps).astype(float)
        #DEBUG
        # cur_joints = self.kinematic.inverse_kinematics(tuple(cur_coordinates))
        # print(np.round(np.degrees(cur_joints), 2))
        accelerating = True
        braking_distance = 0
        for i in range(interpolation_steps):
            cur_coordinates += move_per_step
            # print(np.round(cur_coordinates, 2))
            cur_joints = self.kinematic.inverse_kinematics(tuple(cur_coordinates))
            # print("Joints: ", np.round(np.degrees(cur_joints), 2))
            self.controller.move_steppers(np.degrees(cur_joints), time_to_move)
            self.kinematic.coordinates = cur_coordinates
            move_length_left -= self.interpolation_step_length
            if move_length_left < move_length / 2:
                braking_distance = np.abs((self.controller.start_velocity ** 2 - cur_speed ** 2) / (2 * acceleration))
            if move_length_left < braking_distance:
                accelerating = False
            if accelerating:
                cur_speed += acceleration * time_to_move
                cur_speed = min(speed, cur_speed)
            else:
                cur_speed -= acceleration * time_to_move
                cur_speed = max(self.interpolation_step_length * 2, cur_speed)
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
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(0)))
    # print()
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(15)))

