"""Find the gripper servo's open and closed angles.

Run it, then type into the terminal:
    a number (0-270)  move to that angle
    +  /  -           step by STEP degrees
    o  /  c           try the current GRIPPER_OPEN_ANGLE / GRIPPER_CLOSED_ANGLE
    q                 quit (releases the servo)
Nothing moves until you type something. Put the angles you settle on into constants.py.
"""

from hardware import Servo
from constants import GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE

PIN = 13    # gripper servo pin, as in create_hardware()
STEP = 5    # degrees per + / -. Smaller = finer adjustment

servo = Servo(PIN)
angle = None

print("Type an angle (0-270), + / - to step, o / c = open / closed from constants, q to quit")
while True:
    cmd = input("gripper angle> ").strip()
    if cmd == "q":
        break
    elif cmd == "o":
        angle = GRIPPER_OPEN_ANGLE
    elif cmd == "c":
        angle = GRIPPER_CLOSED_ANGLE
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
    print("gripper at", angle)

servo.servo.deinit()   # stop the PWM signal so the servo goes limp
print("done, last angle:", angle)
