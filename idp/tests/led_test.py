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


