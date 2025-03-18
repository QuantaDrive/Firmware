import threading
import time

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

threading.Thread(target=planner.run, daemon=True).start()

print("Start")
planner.plan_move((300, 0, 525, np.radians(0), np.radians(90),  np.radians(0)))


while True:
    command = input("Command: ").lower()
    if command == "help":
        print("AUTO: Set the planner to automatic mode")
        print("MANUAL: Set the planner to manual mode")
        print("SWAP: Swap the program buffers")
        print("EXIT: Exit the program")
    elif command == "auto":
        planner.set_move_mode(Planner.MoveMode.AUTO)
    elif command == "manual":
        planner.set_move_mode(Planner.MoveMode.MANUAL)
    elif command == "swap":
        planner.swap_program_buffers()
    elif command == "exit":
        break

# input("Press enter to continue")
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