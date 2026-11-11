"""Hardware initialisation and low-level interfaces for the AGV."""

from machine import Pin, PWM


class Motor:
    """Control one DC motor using a direction pin and a PWM speed pin."""

    def __init__(self, dirPin, PWMPin):
        """Configure the direction and PWM pins for one motor."""
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def off(self):
        """Stop the motor by setting its PWM duty cycle to zero."""
        self.pwm.duty_u16(0)

    def Forward(self, speed=100):
        """Rotate the motor forwards at ``speed`` percent."""
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=30):
        """Rotate the motor backwards at ``speed`` percent."""
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))


class Line:
    """Read one digital line sensor."""

    def __init__(self, digitalPin):
        """Configure ``digitalPin`` as the input for this line sensor."""
        self.sensor = Pin(digitalPin, Pin.IN)
    
    def get_value(self):
        """Return the current digital reading from the line sensor."""
        return self.sensor.value()


class Hardware:
    """Group the motor and line-sensor objects used by the AGV."""

    def __init__(self, motor1, motor2, line1, line2, line3, line4):
        """Store the already-configured hardware component objects."""
        self.motor1 = motor1
        self.motor2 = motor2
        self.line1 = line1
        self.line2 = line2
        self.line3 = line3
        self.line4 = line4


def create_hardware():
    """Create the hardware objects using the AGV's physical pin assignments."""
    return Hardware(
        motor1=Motor(4, 5),
        motor2=Motor(6, 7),
        line1=Line(27),
        line2=Line(26),
        line3=Line(22),
        line4=Line(21)
    )
