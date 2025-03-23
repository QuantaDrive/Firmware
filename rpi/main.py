import threading
import time

import numpy as np

from kinematics import Move
from planner import Planner

planner = Planner.from_config("moveo.yaml")
# planner.controller.connect()
# planner.controller.reset()
# planner.controller.send_config()
#
# input("Bring the arm manually to the home position and press enter to start")
# for i in range(6):
#     planner.controller.set_homed(i)
#     planner.controller.set_position(i, 0)

threading.Thread(target=planner.run, daemon=True).start()

print("Start")
move = Move()
move.velocity.cruise_velocity = 25
move.coordinate = np.array([300, 0, 525, 0, 90,  0])
planner.run_move(move)

while True:
    command = input("Command: ").lower()
    if command == "help":
        print("AUTO: Set the planner to automatic mode")
        print("MANUAL: Set the planner to manual mode")
        print("MDI: Go into MDI mode")
        print("LOAD: Load the program from buffer")
        print("EXIT: Exit the program")
    elif command == "auto":
        planner.set_move_mode(Planner.MoveMode.AUTO)
    elif command == "manual":
        planner.set_move_mode(Planner.MoveMode.MANUAL)
    elif command == "load":
        planner.load_program()
    elif command == "exit":
        break