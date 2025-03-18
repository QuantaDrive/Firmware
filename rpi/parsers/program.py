from __future__ import annotations

class Program:
    program_buffer: Program = None

    class ProgramLine:
        def __init__(self, coordinates, speed, relative = False):
            self.coordinates = coordinates
            self.speed = speed
            self.relative = relative

    def __init__(self):
        self.name = ""
        self.lines = []