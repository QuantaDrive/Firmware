from .program import Program
from .settings import Settings

def parse_gcode_MDI():
    coordinate_names = Settings().coordinate_names
    coordinate_name_index: dict[str, int] = {name.upper(): i for i, name in enumerate(coordinate_names)}
    relative = False
    pre_speed = float('inf')
    program_line = None
    while True:
        gcode_command = yield program_line
        program_line = None
        gcode_command = gcode_command.strip().upper()
        if gcode_command.startswith(";"):
            continue
        gcode_tokens = gcode_command.split(";")[0].split(" ")
        if gcode_tokens[0] == "G0":
            coordinates = [None for _ in range(len(coordinate_names))]
            for i in range(1, len(gcode_tokens)):
                coordinate_index = coordinate_name_index[gcode_tokens[i][0]]
                coordinates[coordinate_index] = float(gcode_tokens[i][1:])
            program_line = Program.ProgramLine(coordinates, float('inf'), relative)
        elif gcode_tokens[0] == "G1":
            coordinates = [None for _ in range(len(coordinate_names))]
            speed = pre_speed
            for i in range(1, len(gcode_tokens)):
                if gcode_tokens[i][0] == "F":
                    speed = float(gcode_tokens[i][1:])
                    continue
                coordinate_index = coordinate_name_index[gcode_tokens[i][0]]
                coordinates[coordinate_index] = float(gcode_tokens[i][1:])
            program_line = Program.ProgramLine(coordinates, speed, relative)
            pre_speed = speed
        elif gcode_tokens[0] == "G90":
            relative = False
        elif gcode_tokens[0] == "G91":
            relative = True


def parse_gcode(gcode: str):
    program = Program()
    parse_generator = parse_gcode_MDI()
    parse_generator.send(None)
    for line in gcode.splitlines():
        program_line = parse_generator.send(line)
        if program_line is not None:
            program.lines.append(program_line)
    Program.program_buffer = program
    print("New program loaded in buffer")
