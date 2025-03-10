import time

import numpy as np

from planner import Planner
from controller import Controller

controller = Controller.from_config("moveo.yaml")
controller.connect()
controller.reset()
controller.send_config()

# input("Bring the arm manually to the home position and press enter to start")
for i in range(6):
    controller.set_homed(i)
    controller.set_position(i, 0)

planner = Planner(controller)

print("Start")
for i in range(5):
    controller.move_steppers((1, 0, 0, 0, 0, 0), 0.5, relative=True)


# planner.plan_move((300, 0, 525, np.radians(0), np.radians(0),  np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 575, np.radians(0), np.radians(0),  np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 575, np.radians(0), np.radians(90), np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((350, 0, 575, np.radians(0), np.radians(90), np.radians(0)))
# planner.plan_move((300, 0, 525, np.radians(0), np.radians(90), np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 525, np.radians(90), np.radians(0),  np.radians(90)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 525, np.radians(0), np.radians(0),  np.radians(45)))

print("Done calculating")
while True:
    time.sleep(1)