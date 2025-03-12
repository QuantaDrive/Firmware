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
        min_speed = self.interpolation_step_length * 2
        cur_speed = min_speed
        time_to_move = self.interpolation_step_length / cur_speed

        cur_coordinates = np.array(start_coordinates).astype(float)
        move_length_left = move_length
        move_per_step = ((np.array(end_coordinates) - np.array(start_coordinates)) / interpolation_steps).astype(float)

        accelerating = True
        braking_distance = 0
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
            self.kinematic.coordinates = cur_coordinates
            # Check for acceleration or deceleration
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


if __name__ == "__main__":
    from controller import Controller
    controller = Controller.from_config("moveo.yaml")
    # controller.connect()
    # controller.reset()
    # controller.send_config()

    planner = Planner(controller)

    planner.plan_move((300, 0, 525, np.radians(0), np.radians(0), np.radians(0)))
    print(planner.kinematic.forward_kinematics([ 0.,    np.radians(2.75), np.radians(68.34),  0.,   np.radians(18.91), -0.  ]))
    print()
    planner.plan_move((300, 0, 525, np.radians(90), np.radians(0),  np.radians(0)))
    print(planner.kinematic.forward_kinematics([ 0.,   np.radians(20.02), np.radians(67.71),  0.,   -0.,   np.radians(90)  ]))
    # print()
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(0)))
    # print()
    # planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(25)))

