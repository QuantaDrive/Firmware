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
            self.make_absolute(start_coordinates, program_line_index - 1)
            return self.lines[program_line_index].coordinates, program_line_index

        start_coordinates, program_line_index_searched = self.make_absolute(start_coordinates, program_line_index - 1)

        for i in range(len(self.lines[program_line_index].coordinates)):
            if self.lines[program_line_index].coordinates[i] is None:
                self.lines[program_line_index].coordinates[i] = start_coordinates[i]
            elif start_coordinates[i] is not None:
                self.lines[program_line_index].coordinates[i] += start_coordinates[i]
            else:
                #TODO: doesnt work when relative is not none and start_coordinates is none
                print("Error: start_coordinates is None")
        self.lines[program_line_index].relative = False

        return self.lines[program_line_index].coordinates, program_line_index
