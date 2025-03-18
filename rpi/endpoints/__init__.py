from .jog import BaseJogEndpoint,Gamepad, GamepadJogController

from .program import BaseProgramEndpoint, WebProgramController

# to add more jog controller types just put | between them
# jog_controller_types = GamepadJogController | MpgJogController
jog_controller_types = GamepadJogController

program_endpoints = [
    WebProgramController
]