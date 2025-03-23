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
        self.lines: list[Program.ProgramLine] = []

    def make_absolute(self, start_coordinates, program_line_index = None):
        if program_line_index is None:
            program_line_index = len(self.lines) - 1

        if program_line_index == -1:
            return start_coordinates, program_line_index

        if not self.lines[program_line_index].relative:
            return self.lines[program_line_index].coordinates, program_line_index

        start_coordinates, program_line_index_searched = self.make_absolute(start_coordinates, program_line_index - 1)

        self.lines[program_line_index].coordinates += start_coordinates
        self.lines[program_line_index].relative = False

        if program_line_index_searched == -1:
            return

        self.make_absolute(start_coordinates, program_line_index_searched - 1)
