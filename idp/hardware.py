# Hardware initialization and management
from machine import ADC, Pin, PWM, I2C
from utime import sleep_ms, ticks_diff, ticks_ms
from constants import *
from VL53L0X import VL53L0X   # plain import: hardware.py is a top-level module, not a package


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

class Ir:
    def __init__(self, digitalPin: int):
        """Create an IR receiver on a digital input pin."""
        self.sensor = Pin(digitalPin, Pin.IN)

    def get_value(self):
        """Return the raw pin value: 0 when infrared is seen, 1 when it isn't (active-low)."""
        return self.sensor.value()


class Button:
    def __init__(self, digitalPin: int, on_stop=None):
        """Start/stop push button wired to GND (internal pull-up, so it reads 0 when pressed).

        ``running`` starts True. Every press flips it. The flip happens in an interrupt,
        so it works even while the main code is stuck in a loop or a sleep. When a press
        flips it to False, ``on_stop`` is called straight away (used to cut the motors)."""
        self.running = True
        self.on_stop = on_stop
        self._last_press = ticks_ms()
        self.sensor = Pin(digitalPin, Pin.IN, Pin.PULL_UP)
        self.sensor.irq(trigger=Pin.IRQ_FALLING, handler=self._pressed)

    def _pressed(self, pin):
        """Interrupt handler: runs on each falling edge (button pushed down)."""
        now = ticks_ms()
        if ticks_diff(now, self._last_press) < BUTTON_DEBOUNCE_MS:
            return   # contact bounce from a press we already counted
        self._last_press = now
        self.running = not self.running
        if not self.running and self.on_stop is not None:
            self.on_stop()

    def is_pressed(self):
        """Return True while the button is held down (raw pin read, no toggling)."""
        return self.sensor.value() == 0


class Hall:
    def __init__(self, adcPin: int):
        """DRV5055A2 linear Hall sensor on an ADC pin. Measures the no-magnet (zero) voltage
        straight away, so create it with no magnet nearby (i.e. at startup)."""
        self.sensor = ADC(Pin(adcPin))
        readings = []
        for _ in range(HALL_ZERO_READS):
            readings.append(self.get_volts())
            sleep_ms(HALL_READ_MS)
        readings.sort()
        self.zero = readings[len(readings) // 2]   # median: a few odd readings don't shift it
        print("Hall zero: %.3f V" % self.zero)

    def get_volts(self):
        """Return the sensor output in volts."""
        return self.sensor.read_u16() * 3.3 / 65535

    def pole_of(self, v):
        """Classify one reading ``v`` (volts): "north" if it is more than HALL_THRESHOLD_V
        above the zero, "south" if more than that below, None if in between (no magnet)."""
        if v > self.zero + HALL_THRESHOLD_V:
            return "north"
        if v < self.zero - HALL_THRESHOLD_V:
            return "south"
        return None

class Servo:
    MIN_DUTY = 1638 # 0°
    MID_DUTY = 4915 # 135°
    MAX_DUTY = 8192 # 270°
    def __init__(self, ServoPin):
        """Create a servo controller on a PWM pin."""
        self.servo = PWM(Pin(ServoPin))
        self.servo.freq(50)  # 50 Hz for servo

    def set_angle(self, angle):
        """Set the servo to a specific angle (0–270 degrees)."""
        # Clamp angle
        if angle < 0:
            angle = 0
        if angle > 270:
            angle = 270

        duty = int(Servo.MIN_DUTY + (angle / 270) * (Servo.MAX_DUTY - Servo.MIN_DUTY))
        self.servo.duty_u16(duty)


class Hardware:
    def __init__(self, motor1: Motor, motor2: Motor, line1: Line, line2: Line, line3: Line, line4: Line, ir: Ir, distance_sensor: DistanceSensor, hall: Hall, height_servo: Servo, gripper_servo: Servo = None, button: Button = None):
        """Encapsulate the AGV's sensors."""
        self.motor1 = motor1
        self.motor2 = motor2
        self.line1 = line1
        self.line2 = line2
        self.line3 = line3
        self.line4 = line4
        self.ir = ir
        self.distance_sensor = distance_sensor
        self.hall = hall
        self.height_servo = height_servo
        self.gripper_servo = gripper_servo
        self.button = button
class DistanceSensor:
    def __init__(self, id: int, sda_pin: int, scl_pin: int):
        """Create a distance sensor using the VL53L0X module."""
        i2c_bus: I2C = I2C(id, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.sensor = VL53L0X(i2c_bus)
        self.sensor.set_Vcsel_pulse_period(self.sensor.vcsel_period_type[0], 18)
        self.sensor.set_Vcsel_pulse_period(self.sensor.vcsel_period_type[1], 14)
        self.sensor.start()
    
    def value(self):
        """Return the current distance reading in millimeters."""
        return self.sensor.read()

def create_hardware():
    """Return a Hardware object with the AGV's motors and sensors."""
    return Hardware(
        motor1=Motor(4, 5, LEFT_SPEED_MULTIPLIER),   # left wheel
        motor2=Motor(7, 6), #Second set of pins are reversed!!!
        line1=Line(21, 1), #Pin 32 (27)
        line2=Line(20, 2), #Pin 31 (26)
        line3=Line(19, 3), #Pin 27 (21)
        line4=Line(18, 4), #Pin 26 (20)
        ir=Ir(22), #Pin 29
        distance_sensor=DistanceSensor(1, 10, 11),
        hall=Hall(28), #GP28 / ADC2, Pin 34
        height_servo=Servo(15) , #Pin 15,
        gripper_servo=Servo(13),  #Pin 13
        button=Button(27)  #GP27, wired to GND
    )
