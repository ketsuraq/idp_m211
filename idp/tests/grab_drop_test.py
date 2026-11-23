"""Grab / drop test: type a command, the gripper does it.

  grab  -> AGV.grab():    open, lower, close on the box, lift
  drop  -> AGV.release(): lower, open, lift
  quit  -> stop (Ctrl-C also works)

Each step waits SERVO_MOVE_MS. Angles come from constants.py (HEIGHT_*_ANGLE,
GRIPPER_*_ANGLE): change them there if a move goes too far or not far enough.
The wheels are never driven.

Needs on the Pico: agv.py, hardware.py, constants.py, VL53L0X.py.
"""

from agv import AGV

agv = AGV()
agv.stop()   # make sure the wheels are off

COMMANDS = {"grab": agv.grab, "drop": agv.release}

print("Type grab, drop or quit.")
try:
    while True:
        cmd = input("> ").strip().lower()
        if cmd == "quit":
            break
        if cmd in COMMANDS:
            print(cmd + "...")
            COMMANDS[cmd]()
            print("done")
        elif cmd:
            print("Unknown command. Type grab, drop or quit.")
except KeyboardInterrupt:
    pass
print("Stopped")
