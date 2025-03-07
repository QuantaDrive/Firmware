import time

import numpy as np

from planner import Planner
from controller import Controller

controller = Controller.from_config("moveo.yaml")
controller.connect()
controller.reset()
controller.send_config()

input("Bring the arm manually to the home position and press enter to start")
for i in range(6):
    controller.set_homed(i)
    controller.set_position(i, 0)

planner = Planner(controller)

time.sleep(5)
print("Start")

# cur_speed = controller.start_velocity
# degrees = 0
# while True:
#     degrees += 0.5
#     time_to_move = 0.5 / cur_speed
#     cur_speed += controller.max_accel * time_to_move
#     cur_speed = min(2, cur_speed)
#     controller.move_steppers([0, degrees, degrees, 0, 0, 0], time_to_move)
#     if degrees >= 45:
#         break
#
# while True:
#     degrees -= 0.5
#     time_to_move = 0.5 / cur_speed
#     cur_speed += controller.max_accel * time_to_move
#     cur_speed = min(2, cur_speed)
#     controller.move_steppers([0, degrees, degrees, 0, 0, 0], time_to_move)
#     if degrees <= 0:
#         break
#
# cur_speed = controller.start_velocity
# degrees = 0
# for i in [3, 4, 5]:
#     while True:
#         degrees += 0.5
#         time_to_move = 0.5 / cur_speed
#         cur_speed += controller.max_accel * time_to_move
#         cur_speed = min(controller.max_velocity, cur_speed)
#         controller.move_stepper(i, degrees, time_to_move)
#         if degrees >= 90:
#             break
#
#     while True:
#         degrees -= 0.5
#         time_to_move = 0.5 / cur_speed
#         cur_speed += controller.max_accel * time_to_move
#         cur_speed = min(controller.max_velocity, cur_speed)
#         controller.move_stepper(i, degrees, time_to_move)
#         if degrees <= 0:
#             break


planner.plan_move((301.5, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(0)))
planner.plan_move((350, 0, 452.5, np.radians(0), np.radians(90.0), np.radians(0)))


while True:
    time.sleep(0.5)
