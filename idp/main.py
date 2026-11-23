"""Grab test (see grab_test.py): start in the start box, facing the ramp.

Drives down each branch in grab_test.SCAN_PLAN to the end of the line. A distance reading
below BOX_DETECT_MM on the way means a box: at the end, collect, turn around, deliver to the
depot, go home. Empty branches: turn around at the end, check the next one. No box anywhere: stop at the
end of the last branch.

main.py runs by itself when the Pico powers up, so the countdown gives time to let go.
"""

from agv import AGV
from grab_test import AFTER_DROP, SCAN_PLAN, GrabMission, run
from utime import sleep, sleep_ms


def main():
    agv = AGV()
    agv.stop()


    while True:
        print("Press button to START")

        # Wait until the button is pressed
        while not agv.button_clicked():
            sleep_ms(20)

        print("Starting mission")

        # Create a fresh mission every time
        mission = GrabMission(SCAN_PLAN, after_drop=AFTER_DROP)

        agv.mission_running = True

        try:
            run(agv, mission)

        except KeyboardInterrupt:
            print("Mission stopped by button")

        except Exception as e:
            print("Error:", e)

        finally:
            # Always stop motors
            agv.stop()
            agv.mission_running = False

            print("Finished in state", mission.state)
            print("Checked:", mission.checked)

        # Don't allow a restart until the button is released
        while agv.hw.button.is_pressed():
            sleep_ms(20)

if __name__ == "__main__":
    main()
