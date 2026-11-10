# built in board micropython modules
from machine import Pin
from utime import sleep

pin = Pin(21, Pin.OUT)

print("LED starts flashing...")
while True:
    try:
        pin.value(1)
        sleep(1) # sleep 1sec
        pin.value(0)
        sleep(1)
    except KeyboardInterrupt:
        break
pin.off()
print("Finished.")

