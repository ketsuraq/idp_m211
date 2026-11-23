"""Find the height servo's lowered and raised angles.

Run it, then type into the terminal:
    a number (0-270)  move to that angle
    +  /  -           step by STEP degrees
    d  /  u           try the current HEIGHT_DOWN_ANGLE / HEIGHT_UP_ANGLE
    q                 quit (releases the servo)
Nothing moves until you type something. Put the angles you settle on into constants.py.
Remember: the box can't be lifted more than 20 mm, so "up" should only just clear the table.
"""

from hardware import Servo
from constants import HEIGHT_DOWN_ANGLE, HEIGHT_UP_ANGLE

PIN = 15    # height servo pin, as in create_hardware()
STEP = 5    # degrees per + / -. Smaller = finer adjustment

servo = Servo(PIN)
angle = None

print("Type an angle (0-270), + / - to step, d / u = down / up from constants, q to quit")
while True:
    cmd = input("height angle> ").strip()
    if cmd == "q":
        break
    elif cmd == "d":
        angle = HEIGHT_DOWN_ANGLE
    elif cmd == "u":
        angle = HEIGHT_UP_ANGLE
    elif cmd in ("+", "-"):
        if angle is None:
            print("type an angle first")
            continue
        angle = angle + STEP if cmd == "+" else angle - STEP
    else:
        try:
            angle = int(cmd)
        except ValueError:
            print("?")
            continue
    angle = max(0, min(270, angle))
    servo.set_angle(angle)
    print("height at", angle)

servo.servo.deinit()   # stop the PWM signal so the servo goes limp
print("done, last angle:", angle)
