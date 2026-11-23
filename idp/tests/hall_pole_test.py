"""Hall sensor pole test (GP28 / ADC2). Like hall_sensor.py, but says which pole it sees.

Uses the robot's own rule (Hall.pole_of in hardware.py, the same as AGV.detect_pole), so
what this prints is what the robot will decide at a box:
  - a pole counts once it is seen HALL_CONFIRM_READS readings in a row
Prints a line only when the confirmed result changes: NORTH, SOUTH or none, with the
voltage and how far it is from the zero.

Start it with NO magnet near the sensor: the zero voltage is measured at startup.
Then hold each side of a box's magnet near the sensor. If a pole is missed, lower
HALL_THRESHOLD_V in constants.py; if "none" flickers to a pole with no magnet, raise it.

Needs on the Pico: hardware.py, constants.py, VL53L0X.py (hardware.py imports it).
"""

from hardware import Hall
from constants import HALL_CONFIRM_READS, HALL_READ_MS, HALL_THRESHOLD_V
from utime import sleep_ms

hall = Hall(28)   # GP28 / ADC2. Measures the zero now: keep magnets away
print("Zero: %.3f V, threshold: +/-%.3f V. Bring a magnet up to the sensor." % (hall.zero, HALL_THRESHOLD_V))

candidate = None   # pole on the latest readings, not yet confirmed
count = 0          # readings in a row that gave `candidate`
shown = "start"    # last confirmed result printed (so each change prints once)
lowest = 3.3       # lowest reading seen, reported at the end

try:
    while True:
        v = hall.get_volts()
        lowest = min(lowest, v)
        pole = hall.pole_of(v)
        count = count + 1 if pole == candidate else 1
        candidate = pole
        if count >= HALL_CONFIRM_READS and candidate != shown:
            shown = candidate
            label = candidate.upper() if candidate else "none"
            print("%-5s  %.3f V  (%+.3f V from zero)" % (label, v, v - hall.zero))
        sleep_ms(HALL_READ_MS)
except KeyboardInterrupt:
    print("Stopped. Lowest reading: %.3f V" % lowest)
