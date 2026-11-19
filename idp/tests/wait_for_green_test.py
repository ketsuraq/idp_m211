"""Test for waiting at the traffic light: stop until the green IR signal arrives.

The IR receiver output is active-low. Red: stays high. Green: pulses, about 0.5 s low
then 0.5 s high. wait_for_green() returns once the output has stayed low for
IR_LOW_HOLD_MS, which a short glitch can't do.

Wiring: receiver output on GP22 (physical pin 29).

What to check on the real traffic light:
  - red:   nothing is printed (it must NOT release)
  - green: "GO" is printed within about a second of green starting
  - then it waits for the light to go back to red, and starts again
Press Ctrl-C to stop.
"""

from machine import Pin
from utime import sleep_ms, ticks_ms, ticks_diff

IR_PIN = 22
IR_LOW_HOLD_MS = 200     # a real low lasts ~500 ms, a glitch much less
RED_GAP_MS = 2000        # no low for this long = the light is back to red (green gaps are ~500 ms)
SAMPLE_MS = 10


def wait_for_green(ir):
    """Block until the pin has stayed low (signal) for IR_LOW_HOLD_MS."""
    low_since = None
    while True:
        if ir.value() == 0:                                   # low = signal
            if low_since is None:
                low_since = ticks_ms()                        # a low just started
            elif ticks_diff(ticks_ms(), low_since) >= IR_LOW_HOLD_MS:
                return                                        # held long enough: green
        else:
            low_since = None                                  # went high: start over
        sleep_ms(SAMPLE_MS)


def wait_for_red(ir):
    """Block until there has been no low for RED_GAP_MS (the pulsing has stopped)."""
    last_low = ticks_ms()
    while ticks_diff(ticks_ms(), last_low) < RED_GAP_MS:
        if ir.value() == 0:
            last_low = ticks_ms()
        sleep_ms(SAMPLE_MS)


def main():
    ir = Pin(IR_PIN, Pin.IN, Pin.PULL_UP)
    try:
        while True:
            start = ticks_ms()
            print("Waiting for green...")
            wait_for_green(ir)
            print("GO (after %d ms)" % ticks_diff(ticks_ms(), start))
            wait_for_red(ir)
            print("Signal stopped (red). Starting again.")
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
