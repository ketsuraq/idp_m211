"""High-level movement and sensor interface for the automated guided vehicle."""

from hardware import create_hardware
from state_machine import RobotState
from constants import *


class AGV:
    """Control the AGV's motors, sensors, and current navigation state."""

    def __init__(self, state=None):
        """Create the hardware interface and use a fresh state when none is given."""
        if state is None:
            state = RobotState() # currently doesn't wrok

        self.state = state
        self.hw = create_hardware()

    def forward(self, speed=FORWARD_SPEED):
        """Drive both motors forward at ``speed`` percent."""
        self.hw.motor1.Forward(speed)
        self.hw.motor2.Reverse(speed)

    def turn(self, direction, speed=TURN_SPEED):
        """Turn in the requested ``left`` or ``right`` direction at ``speed`` percent."""
        if direction == "left":
            self.hw.motor1.Forward(speed)
            self.hw.motor2.Forward(speed)

        elif direction == "right":
            self.hw.motor1.Reverse(speed)
            self.hw.motor2.Reverse(speed)

    def correct(self, direction, speed=FORWARD_SPEED):
        # goes to the left
        if direction == "left":
            self.hw.motor1.Forward(speed-CORRECTION_DIFF)
            self.hw.motor2.Reverse(speed)

        # goes to the right
        elif direction == "right":
            self.hw.motor1.Forward(speed)
            self.hw.motor2.Reverse(speed-CORRECTION_DIFF)

    def backward(self, speed=BACKWARD_SPEED):
        """Drive both motors backwards at ``speed`` percent."""
        self.hw.motor1.Reverse(speed)
        self.hw.motor2.Forward(speed)

    def stop(self):
        """Stop both drive motors."""
        self.hw.motor1.off()
        self.hw.motor2.off()

    # assumes 0 is leftmost and 3 is rightmost
    def read_line_sensor(self):
        """Return a list of readings from all 4 line sensors."""
        return [
            self.hw.line1.get_value(),
            self.hw.line2.get_value(),
            self.hw.line3.get_value(),
            self.hw.line4.get_value()
        ]

    def identify_config(self, line_readings):
        if line_readings == [0, 1, 1, 0]:
            return "line"
        elif line_readings == [0, 1, 0, 0]:
            return "left correction"
        elif line_readings == [0, 0, 1, 0]:
            return "right correction"
        elif line_readings == [1, 1, 1, 0] or line_readings == [1, 1, 0, 0]:
            return "left turn"
        elif line_readings == [0, 1, 1, 1] or line_readings == [0, 0, 1, 1]:
            return "right turn"
        elif line_readings == [1, 1, 1, 1]:
            return "parallel"
        elif line_readings == [0, 0, 0, 0]:
            return "line lost"
        else:
            return "unknown"

    def follow_line(self, speed=FORWARD_SPEED):
        readings = self.read_line_sensor()
        config = self.identify_config(readings)

        if config == "line":
            self.forward(speed)

        elif config == "left correction":
            self.correct("left", speed)

        elif config == "right correction":
            self.correct("right", speed)

        return config

    def read_magnet_sensor(self):
        """Return the detected box polarity once the magnetic sensor is connected."""
        pass

    def read_hall_sensor(self):
        """Return whether a box is present once the Hall sensor is connected."""
        pass

    def read_ir_sensor(self):
        """Return the IR traffic-light signal once that sensor is connected."""
        pass
