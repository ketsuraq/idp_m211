class RobotState:
    def __init__(self):
        self.phase = "initial_turn"
        self.junction_count = 0

    def handle_junction(self, config):

        if self.phase == "to_corner":
            if config == "right turn":
                self.phase = "count_junctions"
                self.junction_count = 0
                return "right"

        elif self.phase == "count_junctions":
            if config == "right turn":
                self.junction_count += 1
                print("Junction:", self.junction_count)

                if self.junction_count == 3:
                    self.phase = "to_end"
                    return "right"

            return "straight"

        return "stop"