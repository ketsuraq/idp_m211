# agv class
# declare methods for movement given paramenters

class AGV:

    def __init__ (self, state):
        self.state = state

    def forward(self):
        pass

    def turn(self, direction):
        pass

    def backward(self):
        pass

    def read_line_sensor(self):
        pass
        # returns detected object (line, corner, T-shape, X-shape, etc.)

    def read_magnet_sensor(self):
        pass

    def read_hall_sensor(self):
        pass

    def read_ir_sensor(self):
        pass
