"""Test for waiting at the traffic light: stop until the green IR signal arrives.

Runs the robot's own code: agv.read_ir_sensor() and line_following_test.wait_for_green().
The IR receiver output is active-low. Red: stays high. Green: a series of short lows,
measured at about 74-85 ms each. wait_for_green() returns once the output has stayed low
for IR_LOW_HOLD_MS (constants.py), which a short glitch can't do.

Wiring: receiver output on GP22 (physical pin 29). The motors are never driven here.

What to check on the real traffic light:
  - red:   nothing is printed (it must NOT release)
  - green: "GO" is printed within about a second of green starting
  - then it waits for the light to go back to red, and starts again
Press Ctrl-C to stop.
"""

from agv import AGV
from line_following_test import wait_for_green
from utime import sleep_ms, ticks_ms, ticks_diff

RED_GAP_MS = 2000        # no low for this long = the light is back to red (the gap between
                         # green lows hasn't been measured; raise this if it reports red during green)
SAMPLE_MS = 10


def wait_for_red(agv):
    """Block until there has been no low for RED_GAP_MS (the pulsing has stopped)."""
    last_low = ticks_ms()
    while ticks_diff(ticks_ms(), last_low) < RED_GAP_MS:
        if agv.read_ir_sensor():
            last_low = ticks_ms()
        sleep_ms(SAMPLE_MS)


def main():
    agv = AGV()
    try:
        while True:
            start = ticks_ms()
            print("Waiting for green...")
            wait_for_green(agv)
            print("GO (after %d ms)" % ticks_diff(ticks_ms(), start))
            wait_for_red(agv)
            print("Signal stopped (red). Starting again.")
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        agv.stop()


if __name__ == "__main__":
    main()
