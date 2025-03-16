import numpy as np

from planner import Planner

planner = Planner.from_config("moveo.yaml")
planner.controller.connect()
planner.controller.reset()
planner.controller.send_config()

input("Bring the arm manually to the home position and press enter to start")
for i in range(6):
    planner.controller.set_homed(i)
    planner.controller.set_position(i, 0)

print("Start")

planner.plan_move((300, 0, 525, np.radians(0), np.radians(90),  np.radians(0)))
input("Press enter to continue")
# planner.plan_move((300, 0, 575, np.radians(0), np.radians(0),  np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 575, np.radians(0), np.radians(90), np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((350, 0, 575, np.radians(0), np.radians(90), np.radians(0)))
# planner.plan_move((300, 0, 525, np.radians(0), np.radians(0), np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 525, np.radians(0), np.radians(0), np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 525, np.radians(90), np.radians(0),  np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 525, np.radians(0), np.radians(90),  np.radians(0)))
# input("Press enter to continue")
# planner.plan_move((300, 0, 525, np.radians(0), np.radians(0),  np.radians(90)))

print("Done calculating")
planner.set_move_mode(Planner.MoveMode.MANUAL)
planner.run()