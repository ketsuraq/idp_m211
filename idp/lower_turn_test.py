"""Lower floor test: moving between rack branches without the 180 degree turn.

Start in the start box facing the ramp and press the button. The robot scans the lower
left rack (branches 1-6), then the lower right rack, delivering at most one box from
each (same rules as first_comp.py), then goes home. No ramp, no IR wait.

The difference from first_comp.py is what happens at an EMPTY branch when the next
target is the next branch of the same rack. Instead of turning around (180) and
driving back to the main line, the robot does what the upper deck already does:

    1. reverse (line following) for LOWER_BACKOUT_MS              -> back at the main line
    2. pivot 90 degrees for LOWER_TURN_MS, towards the next branch   (left rack: left,
                                                                     right rack: right)
    3. reverse for LOWER_AFTER_PIVOT_BACK_MS
    4. drive forward to the next branch's junction, turn into it  (left rack: right,
                                                                     right rack: left)

Anything else (a box was taken, or the next target is on another rack) still uses the
normal turn around. The upper deck still uses first_comp's transition, so
pickup_transition can be dropped into first_comp.run later.

Needs on the Pico: this file, first_comp.py, grab_test.py, line_following_test.py,
state_machine.py, constants.py, agv.py, hardware.py, VL53L0X.py.
Run this file directly (it has its own button loop).
"""

import first_comp
from constants import BACKWARD_SPEED, FORWARD_SPEED, TURN_ALIGN_SETTLE_MS, TURN_SPEED
from state_machine import expected_sides, route

# ---- settings (tune on the table) --------------------------------------------------
LOWER_BACKOUT_MS = 1000           # 1. reverse from the branch end back to the main line
LOWER_TURN_MS = 900               # 2. the 90 degree pivot (started at the upper deck's value)
LOWER_AFTER_PIVOT_BACK_MS = 100   # 3. short reverse after the pivot
# 4. keep turning this long after the middle sensors find the branch line (same as upstairs)
LOWER_TURN_SETTLE_MS = TURN_ALIGN_SETTLE_MS + 50

LOWER_RACKS = first_comp.RACKS[:2]    # ("left", 1-6), ("right", 1-6)

# Pivot out of a branch towards the next one, and the turn into the next branch
PIVOT_TURN = {"left": "left", "right": "right"}
INTO_BRANCH_TURN = {"left": "right", "right": "left"}


class LowerMission(first_comp.FirstCompMission):
    """first_comp's mission with only the two lower racks. Goes home after them."""

    def __init__(self, clock=None):
        first_comp.FirstCompMission.__init__(self, clock)
        self.pickups = [node for node in self.pickups if not node.startswith("upper_")]
        self.remaining = route("start", self.pickups[0])   # first junction met is the start T


def lower_transition_plan(current_pickup, next_pickup):
    """(side, branch number) if next_pickup is the next branch on the same lower rack as
    current_pickup, e.g. "left_pickup_2_end" -> "left_pickup_3_end". Otherwise None."""
    for side in ("left", "right"):
        prefix = side + "_pickup_"
        suffix = "_end"
        if current_pickup.startswith(prefix) and current_pickup.endswith(suffix):
            number_text = current_pickup[len(prefix):-len(suffix)]
            if not number_text.isdigit():
                return None
            number = int(number_text)
            if next_pickup == first_comp.pickup_node(side, number + 1):
                return side, number
            return None
    return None


def reverse_for(agv, milliseconds):
    """Reverse (line following) for milliseconds, then stop."""
    from utime import sleep_ms, ticks_diff, ticks_ms

    agv.backward(BACKWARD_SPEED)
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < milliseconds:
        agv.check_stop_button()
        agv.reverse_follow_line(BACKWARD_SPEED)
        sleep_ms(10)
    agv.stop()


def lower_pickup_transition(agv, mission):
    """At the end of an empty lower branch: go straight to the next branch of the same
    rack (steps 1-4 at the top). Returns False, doing nothing, when that doesn't apply,
    so the driver turns around as usual."""
    if not mission.pickups:
        return False
    plan = lower_transition_plan(mission.here, mission.pickups[0])
    if plan is None:
        return False

    side, number = plan
    junction = "%s_pickup_%d" % (side, number)
    next_junction = "%s_pickup_%d" % (side, number + 1)
    if (
        len(mission.remaining) < 2
        or mission.remaining[0] != junction
        or mission.remaining[1] != next_junction
    ):
        return False

    from line_following_test import turn_at_junction
    from utime import sleep_ms, ticks_diff, ticks_ms

    # 1. Back out of the branch to the main line
    print("lower: reversing for", LOWER_BACKOUT_MS, "ms")
    reverse_for(agv, LOWER_BACKOUT_MS)

    # Timed, so nothing is sensed: report this junction as the map expects it
    action = mission.on_junction(expected_sides(junction, mission.heading))
    if action != PIVOT_TURN[side]:
        raise RuntimeError("unexpected turn at %s: %s" % (junction, action))

    # 2. Pivot 90 degrees towards the next branch
    print("lower: pivoting", action, "for", LOWER_TURN_MS, "ms")
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < LOWER_TURN_MS:
        agv.check_stop_button()
        agv.turn(action, TURN_SPEED)
        sleep_ms(10)
    agv.stop()

    # 3. Short reverse
    print("lower: reversing for", LOWER_AFTER_PIVOT_BACK_MS, "ms")
    reverse_for(agv, LOWER_AFTER_PIVOT_BACK_MS)

    # 4. Forward to the next branch's junction, then turn into the branch
    print("lower: driving forward to", next_junction)
    agv.forward(FORWARD_SPEED)
    first_comp.drive_to_lane(agv)   # same lane finder as upstairs
    action = mission.on_junction(expected_sides(next_junction, mission.heading))
    if action != INTO_BRANCH_TURN[side]:
        raise RuntimeError("expected to turn %s into %s, got %s"
                           % (INTO_BRANCH_TURN[side], next_junction, action))
    turn_at_junction(agv, action, settle_ms=LOWER_TURN_SETTLE_MS)
    return True


def pickup_transition(agv, mission):
    """Lower racks: the new transition. Upper deck: first_comp's (unchanged)."""
    if lower_pickup_transition(agv, mission):
        return True
    return first_comp.ramp_pickup_transition(agv, mission)


def run(agv, mission):
    """first_comp's driver with the lower floor transition added."""
    from grab_test import run as grab_test_run

    return grab_test_run(
        agv,
        mission,
        wait_for_green_signal=first_comp.WAIT_FOR_GREEN_SIGNAL,
        ramp_pickup_transition=pickup_transition,
    )


def main():
    """Wait for the button, run the lower floor test, repeat (same loop as main.py)."""
    from agv import AGV
    from utime import sleep_ms

    # Import everything now, so a file missing from the Pico fails here with a traceback
    import grab_test
    import line_following_test
    print("lower_turn_test: modules loaded")

    agv = AGV()
    agv.stop()

    while True:
        print("Press button to START (lower floor turn test)")
        while not agv.button_clicked():
            sleep_ms(20)

        print("Starting lower floor turn test")
        mission = LowerMission()
        agv.mission_running = True

        try:
            run(agv, mission)
        except KeyboardInterrupt:
            print("Mission stopped by button")
        except Exception as e:
            print("Error:", e)
        finally:
            agv.stop()
            agv.stop_mission_lights()
            agv.mission_running = False
            print("Finished in state", mission.state, "| delivered:", mission.delivered)

        while agv.hw.button.is_pressed():
            sleep_ms(20)


if __name__ == "__main__":
    main()
