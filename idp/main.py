"""Grab test (see grab_test.py): start in the start box, facing the ramp.

Drives down each branch in grab_test.SCAN_PLAN to the end of the line. A distance reading
below BOX_DETECT_MM on the way means a box: at the end, collect, turn around, deliver to the
depot, go home. Empty branches: turn around at the end, check the next one. No box anywhere: stop at the
end of the last branch.

main.py runs by itself when the Pico powers up, so the countdown gives time to let go.
"""

from agv import AGV
from grab_test import AFTER_DROP, SCAN_PLAN, GrabMission, run
from utime import sleep


def main():
    agv = AGV()
    mission = GrabMission(SCAN_PLAN, after_drop=AFTER_DROP)

    try:
        print("Starting from start box in:")

        for s in range(3, 0, -1):
            print(s)
            sleep(1)

        run(agv, mission)

    except KeyboardInterrupt:
        print("Ctrl-C")

    except Exception as e:
        print("Error:", e)

    finally:
        agv.stop()
        print("Finished in state", mission.state, "| checked:", mission.checked)


if __name__ == "__main__":
    main()
