"""Print the four line sensors as a list, e.g. [0, 1, 1, 0], leftmost first.

Same pins and order as create_hardware() / agv.read_line_sensor(), so what you see here
is exactly what the robot code sees. Ctrl-C to stop.
"""

from hardware import Line
from utime import sleep_ms

PRINT_MS = 100   # time between prints. Smaller = more responsive, but scrolls faster

# Same as create_hardware(): GPIO number, sensor number
line1 = Line(21, 1)   # leftmost
line2 = Line(20, 2)
line3 = Line(19, 3)
line4 = Line(18, 4)   # rightmost

try:
    while True:
        print([line1.get_value(), line2.get_value(), line3.get_value(), line4.get_value()])
        sleep_ms(PRINT_MS)
except KeyboardInterrupt:
    print("Stopped")
