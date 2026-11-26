"""Upper rack test: the competition code (first_comp.py) without the lower racks.

Start in the start box facing the ramp and press the button. The robot drives to the
start T, waits there for the IR green signal, goes up the ramp and scans upper left, then
upper right, then goes home.

Everything, including the upper deck moves and their timings (UPPER_SIDE), is in
first_comp.py, so this tests exactly what main.py runs in the competition. Tune there.

To run it without the USB cable, point main.py at it:
    from upper_test import UpperMission, run   /   mission = UpperMission()
then upload, unplug and power-cycle. (Switch main.py back to first_comp afterwards.)

Needs on the Pico: this file, first_comp.py, grab_test.py, line_following_test.py,
state_machine.py, constants.py, agv.py, hardware.py, VL53L0X.py.

Helloooo
"""

import first_comp
from first_comp import run      # main.py imports run from here
from state_machine import route


class UpperMission(first_comp.FirstCompMission):
    """FirstCompMission with only the upper racks: start -> IR wait -> ramp -> upper left 1."""

    def __init__(self, clock=None):
        first_comp.FirstCompMission.__init__(self, clock)
        self.pickups = [node for node in self.pickups if node.startswith("upper_")]
        self.remaining = route("start", self.pickups[0])   # first junction met is the start T


def main():
    """main.py's loop, with UpperMission instead of FirstCompMission."""
    from agv import AGV
    from utime import sleep_ms

    # Import everything now, so a file missing from the Pico fails here with a traceback
    import grab_test
    import line_following_test
    print("upper_test: modules loaded")

    agv = AGV()
    agv.stop()

    while True:
        print("Press button to START (upper racks only)")

        # Wait until the button is pressed
        while not agv.button_clicked():
            sleep_ms(20)

        print("Starting upper rack test")

        # Create a fresh mission every time (this also starts its clock)
        mission = UpperMission()

        agv.mission_running = True   # run() starts the amber flashing

        try:
            run(agv, mission)

        except KeyboardInterrupt:
            print("Mission stopped by button")

        except Exception as e:
            print("Error:", e)

        finally:
            # Always stop motors, and switch every LED off
            agv.stop()
            agv.stop_mission_lights()
            agv.mission_running = False

            print("Finished in state", mission.state)
            print("Delivered:", mission.delivered)

        # Don't allow a restart until the button is released
        while agv.hw.button.is_pressed():
            sleep_ms(20)


if __name__ == "__main__":
    main()
