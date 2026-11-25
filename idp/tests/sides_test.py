"""Side branches test: competition_1's run, on the lower sides only, once each.

What it does: lower left branches 1-6, then lower right branches 1-6, using exactly
competition_1's rules (way out, way back, closest depot if no pole is read; see the top of
competition_1.py). Each side delivers at most one box. After lower right it goes home to
the start box. It never goes up the ramp.

Run: start the robot in the start box facing the ramp, then run this file.
Needs on the Pico: sides_test.py, competition_1.py, state_machine.py, constants.py,
agv.py, hardware.py, VL53L0X.py.
"""

from agv import AGV
from competition_1 import CompetitionMission, run

# Sides to scan, in order: (map name prefix, number of branches). Remove one to test a
# single side, e.g. [("right", 6)].
TEST_SIDES = [
    ("left", 6),
    ("right", 6),
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
