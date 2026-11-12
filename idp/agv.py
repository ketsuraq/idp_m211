"""High-level movement and sensor interface for the automated guided vehicle."""

from hardware import create_hardware
from constants import *
from utime import sleep_ms


class AGV:
    """Control the AGV's motors and sensors. Navigation decisions live in state_machine.Mission."""

    def __init__(self):
        """Create the hardware interface."""
        self.hw = create_hardware()

    def forward(self, speed=FORWARD_SPEED):
        """Drive both motors forward at ``speed`` percent."""
        self.hw.motor1.Forward(speed)
        self.hw.motor2.Forward(speed)

    def turn(self, direction, speed=TURN_SPEED):
        """Turn in the requested ``left`` or ``right`` direction at ``speed`` percent."""
        if direction == "left":
            self.hw.motor1.Reverse(speed)
            self.hw.motor2.Forward(speed)

        elif direction == "right":
            self.hw.motor1.Forward(speed)
            self.hw.motor2.Reverse(speed)

    def correct(self, direction, speed=FORWARD_SPEED):
        # goes to the left
        if direction == "left":
            self.hw.motor1.Forward(speed)
            self.hw.motor2.Forward(speed+CORRECTION_DIFF)

        # goes to the right
        elif direction == "right":
            self.hw.motor1.Forward(speed+CORRECTION_DIFF)
            self.hw.motor2.Forward(speed)

    def backward(self, speed=BACKWARD_SPEED):
        """Drive both motors backwards at ``speed`` percent."""
        self.hw.motor1.Reverse(speed)
        self.hw.motor2.Reverse(speed)

    def reverse_correct(self, direction, speed=BACKWARD_SPEED):
        """While reversing, swing the front (where the sensors are) to ``direction``.
        Front swings left when the left wheel reverses faster than the right."""
        if direction == "left":
            self.hw.motor1.Reverse(speed+CORRECTION_DIFF)
            self.hw.motor2.Reverse(speed)

        elif direction == "right":
            self.hw.motor1.Reverse(speed)
            self.hw.motor2.Reverse(speed+CORRECTION_DIFF)

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
            sleep_ms(100)

        elif config == "right correction":
            self.correct("right", speed)
            sleep_ms(100)

        return config

    def reverse_follow_line(self, speed=BACKWARD_SPEED):
        """Same as follow_line, but driving backwards. The sensors are at the front, so a
        line under the left sensor still means "swing the front left"."""
        readings = self.read_line_sensor()
        config = self.identify_config(readings)

        if config == "line":
            self.backward(speed)

        elif config == "left correction":
            self.reverse_correct("left", speed)
            sleep_ms(100)

        elif config == "right correction":
            self.reverse_correct("right", speed)
            sleep_ms(100)

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

