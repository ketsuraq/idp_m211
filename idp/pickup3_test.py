"""Mid-route test: start just past the upper green depot junction (right_depot_2).

Place the robot on the right-hand main line, facing N (towards the ramp), with the sensors
on the line a little past the right_depot_2 branch. It then runs the rest of the first test:
right pickup 1 (straight) -> pickup 2 (straight) -> pickup 3 (left) -> (collect) -> turn around
-> upper green depot -> (drop) -> turn around -> back to the start box.

Same driving code as line_following_test.py (run() with from_box=False).
See docs/turnaround_test.md.
"""

from agv import AGV
from line_following_test import PICKUPS, run
from state_machine import Mission, route
from utime import sleep

START_AFTER = "right_depot_2"   # junction the robot has just passed
START_HEADING = "N"


def main():
    agv = AGV()
    mission = Mission(PICKUPS)
    # Start mid-route instead of in the box
    mission.here = START_AFTER
    mission.heading = START_HEADING
    mission.remaining = route(START_AFTER, PICKUPS[0])[1:]
    try:
        print("Starting after %s facing %s in:" % (START_AFTER, START_HEADING))
        for s in range(3, 0, -1):
            print(s)
            sleep(1)
        run(agv, mission, from_box=False)
    except KeyboardInterrupt:
        print("Ctrl-C")
    except Exception as e:
        print("Error:", e)
    finally:
        agv.stop()
        print("Finished in state", mission.state)


if __name__ == "__main__":
    main()
