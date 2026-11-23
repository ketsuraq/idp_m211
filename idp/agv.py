"""High-level movement and sensor interface for the automated guided vehicle."""

from hardware import create_hardware
from constants import *
from utime import sleep_ms, ticks_diff, ticks_ms


class AGV:
    """Control the AGV's motors and sensors. Navigation decisions live in state_machine.Mission."""

    def __init__(self):
        """Create the hardware interface."""
        self.hw = create_hardware()

        self.last_button_press = ticks_ms()
        self.button_was_pressed = False
        self.mission_running = False

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

    def button_clicked(self):
        """Return True once for each debounced button press."""
        pressed = self.hw.button.pressed()

        if pressed and not self.button_was_pressed:
            self.button_was_pressed = True

            if ticks_diff(ticks_ms(), self.last_button_press) > 250:
                self.last_button_press = ticks_ms()
                return True

        elif not pressed:
            self.button_was_pressed = False

        return False


    def check_stop_button(self):
        """Stop the robot if the button is pressed during a mission."""
        if self.mission_running and self.button_clicked():
            self.stop()
            raise KeyboardInterrupt

    def interruptible_sleep(self, milliseconds):
        """Sleep while checking whether the stop button was pressed."""
        start = ticks_ms()

        while ticks_diff(ticks_ms(), start) < milliseconds:
            self.check_stop_button()
            sleep_ms(10)

    # assumes 0 is leftmost and 3 is rightmost
    def read_line_sensor(self):
        """Return a list of readings from all 4 line sensors."""
        return [
            self.hw.line1.get_value(),
            self.hw.line2.get_value(),
            self.hw.line3.get_value(),
            self.hw.line4.get_value()
        ]

    def read_distance_sensor(self):
        """Return the current distance reading in millimeters from the VL53L0X sensor."""
        return self.hw.distance_sensor.value()

    def centred_on_line(self):
        """Check only the two middle line sensors."""
        readings = self.read_line_sensor()
        return readings[1] == 1 and readings[2] == 1

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

    def detect_pole(self):
        """Call at the box. Returns "south" or "north" once
        the same pole is seen HALL_CONFIRM_READS readings in a row, HALL_READ_MS apart, or
        None if that doesn't happen within HALL_TIMEOUT_MS."""
        hall = self.hw.hall
        last = None
        count = 0
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < HALL_TIMEOUT_MS:
            self.check_stop_button()

            v = hall.get_volts()
            if v >= HALL_GLITCH_V:   # below: loose-contact glitch, ignore it
                if v > hall.zero + HALL_THRESHOLD_V:
                    pole = "south"
                elif v < hall.zero - HALL_THRESHOLD_V:
                    pole = "north"
                else:
                    pole = None
                count = count + 1 if pole is not None and pole == last else (1 if pole else 0)
                last = pole
                if count >= HALL_CONFIRM_READS:
                    return pole
            self.interruptible_sleep(HALL_READ_MS)
        return None

    def read_ir_sensor(self):
        """Return True while the IR receiver sees the traffic light's signal.
        The receiver is active-low: its output pin goes low when it sees the signal."""
        return self.hw.ir.get_value() == 0

