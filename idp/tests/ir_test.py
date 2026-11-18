from machine import Pin
from utime import sleep_ms

ir = Pin(22, Pin.IN)
last = None

try:
    while True:
        value = ir.value()

        if value != last:
            if value == 0:
                print("IR: signal")
            else:
                print("IR: none")
            last = value

        sleep_ms(10)
except KeyboardInterrupt:
    print("Stopped")
