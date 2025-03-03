import controller

speed = 10          # mm per second
max_speed = 600     # mm per second
acceleration = 50   # mm per second^2

controller = controller.load_controller_config("moveo.yaml")
controller.connect()
controller.reset()
controller.send_config()

distance = 1
while True:
    time_to_move = distance / speed
    speed += acceleration * time_to_move
    speed = min(max_speed, speed)
    controller.move_steppers([1, distance], time_to_move, relative=True)
