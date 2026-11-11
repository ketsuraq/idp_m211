# Hardware initialization and management
from machine import Pin, PWM


class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed=100):
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=30):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

class Line:
    def __init__(self, digitalPin):
        self.sensor = Pin(digitalPin, Pin.IN)
    
    def get_value(self):
        return self.sensor.value()

class Hardware:
    def __init__(self, motor1, motor2, motor3, motor4, line1, line2, line3, line4):
        self.motor1 = motor1
        self.motor2 = motor2
        self.line1 = line1
        self.line2 = line2
        self.line3 = line3
        self.line4 = line4


def create_hardware():
    # Initialise actual hardware
    return Hardware(
        motor1=Motor(4, 5),
        motor2=Motor(6, 7),
        line1=Line(27),
        line2=Line(26),
        line3=Line(22),
        line4=Line(21)
    )