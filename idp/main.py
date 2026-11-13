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




def main():
    agv = AGV()
    mission = Mission(PICKUPS)

    try:
        print("Starting from start box in:")

        for s in range(3, 0, -1):
            print(s)
            sleep(1)

        run(agv, mission, from_box=True)

    except KeyboardInterrupt:
        print("Ctrl-C")

    except Exception as e:
        print("Error:", e)

    finally:
        agv.stop()
        print("Finished in state", mission.state)



if __name__ == "__main__":
    main()
