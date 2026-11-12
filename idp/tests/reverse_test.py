"""Reversing-out test.

Place the robot at the END of a branch, facing the dead end (e.g. right pickup 3).
It reverses (line following backwards) until the sensors reach the main line,
pivots towards PIVOT, then follows the new line for FOLLOW_MS and stops.

Tune with this: BACKWARD_SPEED, REVERSE_TURN_TIME, REVERSE_FORWARD_TIME (constants.py).
"""

from agv import AGV
from constants import *
from line_following_test import reverse_to_main_line, pivot_to_line
from utime import sleep, sleep_ms, ticks_ms, ticks_diff

PIVOT = "left"       # right side of the arena: "left" heads back towards the start box
FOLLOW_MS = 3000


def main():
    agv = AGV()
    try:
        print("Put robot at the end of a branch. Starting in:")
        for s in range(3, 0, -1):
            print(s)
            sleep(1)

        seen = reverse_to_main_line(agv)
        print("Main line reached, sensors saw:", seen, "(expected: both)")

        pivot_to_line(agv, PIVOT)
        print("Pivot done, following line")

        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < FOLLOW_MS:
            agv.follow_line(FORWARD_SPEED)
            sleep_ms(10)
    except KeyboardInterrupt:
        print("Ctrl-C")
    except Exception as e:
        print("Error:", e)
    finally:
        agv.stop()
        print("Stopped")


if __name__ == "__main__":
    main()
