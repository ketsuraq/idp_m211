"""Connection test for a DFRobot Gravity digital line-tracking sensor.

Wiring (one sensor head):
    Sensor GND / black  -> Pico GND
    Sensor VCC / red     -> Pico 3V3(OUT)
    Sensor SIG / green   -> Pico GP2

For a second sensor head, connect its SIG pin to GP3.  Change SENSOR_PINS
below if different GPIOs are used.

Important: Pico GPIO pins are 3.3 V only.  Power the sensor from 3V3 when
its output is connected directly to the Pico.  Do not connect a 5 V output
to a Pico GPIO pin; use a level shifter or voltage divider instead.

For the standard Gravity tracker, LOW (0) means a black line is detected and
HIGH (1) means the sensor sees the lighter background.  Flip LINE_IS_LOW if
your particular module has the opposite logic.
"""

from machine import Pin
from time import sleep_ms


# Pico GPIO numbers, not physical pin numbers.
SENSOR_PINS = (21, 22, 26, 27)  # Use (2,) if testing only one sensor.
LINE_IS_LOW = True
SAMPLE_PERIOD_MS = 200


def is_line_detected(raw_value):
    """Return True when the configured sensor logic reports a line."""
    return raw_value == 0 if LINE_IS_LOW else raw_value == 1


def describe_reading(raw_value):
    if is_line_detected(raw_value):
        return "LINE / DARK SURFACE"
    return "BACKGROUND / LIGHT SURFACE"


def main():
    sensors = [Pin(pin_number, Pin.IN) for pin_number in SENSOR_PINS]
    previous = [None] * len(sensors)

    print("Gravity line-tracker connection test")
    print("Move each sensor between the line and the background.")
    print("Press Ctrl-C to stop.\n")

    try:
        while True:
            for index, sensor in enumerate(sensors):
                raw_value = sensor.value()

                # Print changes immediately, plus the first observed value.
                if raw_value != previous[index]:
                    print(
                        "Sensor {} (GP{}): raw={}  {}".format(
                            index + 1,
                            SENSOR_PINS[index],
                            raw_value,
                            describe_reading(raw_value),
                        )
                    )
                    previous[index] = raw_value

            sleep_ms(SAMPLE_PERIOD_MS)
    except KeyboardInterrupt:
        print("\nFinished.")


if __name__ == "__main__":
    main()
