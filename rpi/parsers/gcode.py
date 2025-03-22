from .program import Program
from .settings import Settings

def parse_gcode(gcode: str):
    program = Program()
    coordinate_names = Settings().coordinate_names
    coordinate_name_index: dict[str, int] = {name.upper(): i for i, name in enumerate(coordinate_names)}
    relative = False
    pre_speed = float('inf')
    for line in gcode.splitlines():
        line = line.strip().upper()
        if line.startswith(";"):
            continue
        line_tokens = line.split(";")[0].split(" ")
        if line_tokens[0] == "G0":
            coordinates = [None for _ in range(len(coordinate_names))]
            for i in range(1, len(line_tokens)):
                coordinate_index = coordinate_name_index[line_tokens[i][0]]
                coordinates[coordinate_index] = float(line_tokens[i][1:])
            program.lines.append(Program.ProgramLine(coordinates, float('inf'), relative))
        elif line_tokens[0] == "G1":
            coordinates = [None for _ in range(len(coordinate_names))]
            speed = pre_speed
            for i in range(1, len(line_tokens)):
                if line_tokens[i][0] == "F":
                    speed = float(line_tokens[i][1:])
                    continue
                coordinate_index = coordinate_name_index[line_tokens[i][0]]
                coordinates[coordinate_index] = float(line_tokens[i][1:])
            program.lines.append(Program.ProgramLine(coordinates, speed, relative))
            pre_speed = speed
        elif line_tokens[0] == "G90":
            relative = False
        elif line_tokens[0] == "G91":
            relative = True
    Program.program_buffer = program
    print("New program loaded in buffer")
