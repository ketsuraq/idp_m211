"""Start/stop button test (GP27, wired to GND).

Uses the real Button class from hardware.py, so it tests the toggle and debounce as well
as the wiring. Prints:
  - "pin: down" / "pin: up"   raw pin changes, to check the wiring
  - "running: True/False"     each time a press flips the state (should be once per press)
  - "STOP callback"           each time a press flips it to not running (later this cuts the motors)

If one press flips the state twice, raise BUTTON_DEBOUNCE_MS in constants.py.
If "pin" never changes, check the wiring: the button should connect GP27 to GND.

Needs on the Pico: hardware.py, constants.py, VL53L0X.py (hardware.py imports it).
"""

from hardware import Button
from utime import sleep_ms

stops = 0


def on_stop():
    """Stand-in for the motor cut-off: count and report each stop."""
    global stops
    stops += 1
    print("STOP callback")


button = Button(27, on_stop=on_stop)   # GP27
last_pin = button.is_pressed()
last_running = button.running
presses = 0

print("running:", button.running, "(should start True). Press the button.")

try:
    while True:
        pin = button.is_pressed()
        if pin != last_pin:
            print("pin:", "down" if pin else "up")
            last_pin = pin

        if button.running != last_running:
            presses += 1
            print("running:", button.running, "| toggles:", presses, "| stops:", stops)
            last_running = button.running

        sleep_ms(10)
except KeyboardInterrupt:
    print("Stopped. Toggles:", presses, "| stops:", stops)
