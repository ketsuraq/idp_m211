# built in board micropython modules
from machine import Pin
from utime import sleep

from hardware import create_hardware
from utime import sleep

hw = create_hardware()
for led in (hw.led_red, hw.led_amber, hw.led_green):
    led.on()
    sleep(1)
    led.off()



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

