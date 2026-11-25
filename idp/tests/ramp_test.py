"""Ramp test: competition_1's run, on the upper deck only, once each side.

What it does: waits in the start box for green, goes up the ramp, scans upper left
branches 1-7, delivers what it finds (down the ramp to the depot), comes back, waits for
green again, scans upper right 1-7, delivers, then goes home to the start box. Same rules
as competition_1 (see the top of competition_1.py). It never checks the lower branches.

Run: start the robot in the start box facing the ramp, then run this file. Have the
traffic light running (or be ready to trigger green), or it waits in the box forever.
Needs on the Pico: ramp_test.py, competition_1.py, state_machine.py, constants.py,
agv.py, hardware.py, VL53L0X.py.
"""

from agv import AGV
from competition_1 import CompetitionMission, run
from state_machine import UPPER_CROSSINGS

# Sides to scan, in order: (map name prefix, number of branches). Branch 7 is the top T.
# Remove one to test a single side, e.g. [("upper_right", UPPER_CROSSINGS + 1)].
TEST_SIDES = [
    ("upper_left", UPPER_CROSSINGS + 1),
    ("upper_right", UPPER_CROSSINGS + 1),
]


def main():
    agv = AGV()
    mission = CompetitionMission(sides=TEST_SIDES, one_pass=True)
    try:
        run(agv, mission)
    except KeyboardInterrupt:
        print("Ctrl-C")
    except Exception as e:
        print("Error:", e)
    finally:
        agv.stop()
        print("Finished in state", mission.state, "| delivered:", mission.delivered)


if __name__ == "__main__":
    main()
