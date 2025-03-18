from parsers.program import Program


def parse_gcode(gcode: str, coordinate_names) -> Program:
    program = Program()
    coordinate_name_index: dict[str, int] = {name.upper(): i for i, name in enumerate(coordinate_names)}
    pre_speed = float('inf')
    for line in gcode.splitlines():
        line = line.strip().upper()
        line_tokens = line.split(";")[0].split(" ")
        if line.startswith(";"):
            continue
        if line_tokens[0] == "G0":
            coordinates = [0 for _ in range(len(coordinate_names))]
            for i in range(1, len(line_tokens)):
                coordinate_index = coordinate_name_index[line_tokens[i][0]]
                coordinates[coordinate_index] = float(line_tokens[i][1:])
            program.lines.append(Program.ProgramLine(coordinates, float('inf')))
        elif line_tokens[0] == "G1":
            coordinates = [0 for _ in range(len(coordinate_names))]
            speed = pre_speed
            for i in range(1, len(line_tokens)):
                if line_tokens[i][0] == "F":
                    speed = float(line_tokens[i][1:])
                coordinate_index = coordinate_name_index[line_tokens[i][0]]
                coordinates[coordinate_index] = float(line_tokens[i][1:])
            program.lines.append(Program.ProgramLine(coordinates, float('inf')))
            pre_speed = speed
    return program
