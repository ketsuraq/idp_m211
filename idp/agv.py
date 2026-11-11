from hardware import create_hardware

class AGV:

    def __init__(self, state):
        self.state = state
        self.hw = create_hardware()

    def forward(self, speed=100):
        self.hw.motor1.Forward(speed)
        self.hw.motor2.Forward(speed)

    def turn(self, direction, speed=30):
        if direction == "left":
            self.hw.motor1.Reverse(speed)
            self.hw.motor2.Forward(speed)

        elif direction == "right":
            self.hw.motor1.Forward(speed)
            self.hw.motor2.Reverse(speed)

    def backward(self, speed=50):
        self.hw.motor1.Reverse(speed)
        self.hw.motor2.Reverse(speed)

    def stop(self):
        self.hw.motor1.off()
        self.hw.motor2.off()

    def read_line_sensor(self):
        pass

    def read_magnet_sensor(self):
        pass

    def read_hall_sensor(self):
        pass

    def read_ir_sensor(self):
        pass
