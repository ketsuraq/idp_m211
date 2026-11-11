"""High-level movement and sensor interface for the automated guided vehicle."""

from hardware import create_hardware
from state_machine import RobotState



class AGV:
    """Control the AGV's motors, sensors, and current navigation state."""

    def __init__(self, state=None):
        """Create the hardware interface and use a fresh state when none is given."""
        if state is None:
            state = RobotState() # currently doesn't wrok

        self.state = state
        self.hw = create_hardware()

    def forward(self, speed=100):
        """Drive both motors forward at ``speed`` percent."""
        self.hw.motor1.Forward(speed)
        self.hw.motor2.Reverse(speed)

    def turn(self, direction, speed=30):
        """Turn in the requested ``left`` or ``right`` direction at ``speed`` percent."""
        if direction == "left":
            self.hw.motor1.Forward(speed)
            self.hw.motor2.Forward(speed)

        elif direction == "right":
            self.hw.motor1.Reverse(speed)
            self.hw.motor2.Reverse(speed)

    def backward(self, speed=50):
        """Drive both motors backwards at ``speed`` percent."""
        self.hw.motor1.Reverse(speed)
        self.hw.motor2.Forward(speed)

    def stop(self):
        """Stop both drive motors."""
        self.hw.motor1.off()
        self.hw.motor2.off()

    def read_line_sensor(self, identifier: int):
        """Return line-sensor readings based on which line sensor is inputted"""
        if identifier == 1:
            return self.hw.line1.get_value()
        elif identifier == 2:
            return self.hw.line2.get_value()
        elif identifier == 3:
            return self.hw.line3.get_value()
        elif identifier == 4:
            return self.hw.line4.get_value()
        else:
            raise ValueError("Invalid line sensor identifier")

    def read_magnet_sensor(self):
        """Return the detected box polarity once the magnetic sensor is connected."""
        pass

    def read_hall_sensor(self):
        """Return whether a box is present once the Hall sensor is connected."""
        pass

    def read_ir_sensor(self):
        """Return the IR traffic-light signal once that sensor is connected."""
        pass
