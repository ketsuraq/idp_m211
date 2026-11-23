"""Print GP27's raw state, outputting 1 or 0 only when the state changes."""

from machine import Pin
from utime import sleep_ms

button = Pin(27, Pin.IN, Pin.PULL_UP)
last_state = button.value()
print(last_state)

try:
    while True:
        state = button.value()
        if state != last_state:
            print(state)
            last_state = state
        sleep_ms(10)
except KeyboardInterrupt:
    pass
