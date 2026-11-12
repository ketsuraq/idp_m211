# Hardware initialization and management
from machine import Pin, PWM
from constants import *


class Motor:
    def __init__(self, dirPin: int, PWMPin: int, multiplier=1.0):
        """Create a motor controller with a direction pin and a PWM pin."""
        self.multiplier = multiplier   # scales every speed sent to this motor
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def off(self):
        """Turn off the motor by setting its PWM duty cycle to 0."""
        self.pwm.duty_u16(0)

    def Forward(self, speed=FORWARD_SPEED):
        """Drive the motor forward at ``speed`` percent."""
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * min(100, speed * self.multiplier) / 100))

    def Reverse(self, speed=FORWARD_SPEED):
        """Drive the motor backward at ``speed`` percent."""
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * min(100, speed * self.multiplier) / 100))

class Line:
    def __init__(self, digitalPin: int, identifier: int):
        """Create a line sensor with a digital input pin and an identifier."""
        self.sensor = Pin(digitalPin, Pin.IN)
        self.identifier = identifier
    
    def get_value(self):
        """Return the current digital value of the line sensor."""
        return self.sensor.value()

class Hardware:
    def __init__(self, motor1: Motor, motor2: Motor, line1: Line, line2: Line, line3: Line, line4: Line):
        """Encapsulate the AGV's sensors."""
        self.motor1 = motor1
        self.motor2 = motor2
        self.line1 = line1
        self.line2 = line2
        self.line3 = line3
        self.line4 = line4


def create_hardware():
    """Return a Hardware object with the AGV's motors and sensors."""
    return Hardware(
        motor1=Motor(4, 5, LEFT_SPEED_MULTIPLIER),   # left wheel
        motor2=Motor(7, 6), #Second set of pins are reversed!!!
        line1=Line(27, 1), #Pin 32
        line2=Line(26, 2), #Pin 31
        line3=Line(21, 3), #Pin 27
        line4=Line(20, 4) #Pin 26
    )
