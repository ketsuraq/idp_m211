"""LED test, using the same AGV methods as a real mission. The motors are never driven.

What you should see:
  1. red, amber, green each on for 1 s, one after another (checks the wiring)
  2. amber flashing once a second (AMBER_FLASH_MS) for 6 s. While it flashes:
     red steady for 3 s (carrying a north-pole box), then green steady for 3 s (south pole).
     Amber should keep flashing evenly throughout: it runs on a timer, not in this loop.
  3. everything off
Press Ctrl-C to stop early; the LEDs are switched off either way.
"""

from agv import AGV
from utime import sleep

agv = AGV()
hw = agv.hw

try:
    print("1. each LED in turn")
    for name, led in (("red", hw.led_red), ("amber", hw.led_amber), ("green", hw.led_green)):
        print("  ", name)
        led.on()
        sleep(1)
        led.off()

    print("2. amber flashing (mission running)")
    agv.start_mission_lights()
    print("   + red steady (north pole)")
    agv.show_polarity("N")
    sleep(3)
    print("   + green steady (south pole)")
    agv.show_polarity("S")
    sleep(3)
except KeyboardInterrupt:
    print("Stopped")
finally:
    agv.stop_mission_lights()
    print("3. all off")
