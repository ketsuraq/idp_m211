from machine import ADC, Pin
from utime import sleep_ms

hall = ADC(Pin(28))

try:
    while True:
        print(f"{hall.read_u16() * 3.3 / 65535:.3f} V")
        sleep_ms(200)
except KeyboardInterrupt:
    print("Stopped")
