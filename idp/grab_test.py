"""Grab test: find a box by driving down each pickup branch in turn.

For every branch on SCAN_PLAN the robot drives down it, watching the distance sensor:
  - reading below BOX_DETECT_MM on the way -> a box is there. It still drives to the end
    of the line (so every branch ends in the same place and the turn around behaves the
    same), then collects, turns around, delivers to the depot and goes home
    (AFTER_DROP = "home")
  - no reading below BOX_DETECT_MM -> empty: turn around at the end, back to the main
    line, next branch
When every branch on one side is checked it moves on to the next side in SCAN_PLAN.
If no branch has a box, it stops at the end of the last one.

The mission (GrabMission) lives here, not in state_machine.py, because it's only for this
progress test. It subclasses Mission, which already handles "box found -> depot,
empty -> next pickup"; GrabMission only adds the rules for this test. The driving moves
are imported from line_following_test.py.

Run: start the robot in the start box facing the ramp, as for line_following_test.py.
Needs these files on the Pico: grab_test.py, line_following_test.py, state_machine.py,
constants.py, agv.py, hardware.py, VL53L0X.py.
"""

from agv import AGV
from constants import *
from line_following_test import (
    back_off_line, check_timeout, drive_to_depot, drive_to_junction, drop, follow_to_end,
    pass_junction,
    recheck, reverse_out_of_depot, stop_in_box, turn_around, turn_at_junction,
)
from state_machine import DEPOT_FOR, FALLBACK_DEPOT, OPPOSITE, Mission, expected_sides
from utime import sleep, sleep_ms, ticks_diff, ticks_ms

# ---- test settings ----------------------------------------------------------------
# Branches to check, side by side, in this order. Branch 1 is nearest the corner.
# To check more, extend the lists or uncomment the left side.
SCAN_PLAN = [
    ("right", [1, 2, 3, 4, 5, 6]),
    #("left", [1, 2, 3]),
]
# After delivering a box: "home" = back to the start box (this test),
# "continue" = carry on with the next unchecked branch.
AFTER_DROP = "home"

# ---- distance sensor settings -----------------------------------------------------
# While driving down a branch, a reading below this (mm) means there's a box in it.
# The robot doesn't stop for it, it only notes it. Higher = sees the box from further
# away, but is more likely to be set off by something else (e.g. the end wall).
BOX_DETECT_MM = 150
# The reading must be below BOX_DETECT_MM this many times in a row, so a single bad
# reading doesn't count as a box. Higher = fewer false boxes, but a box seen only
# briefly might be missed.
BOX_CONFIRM_READS = 2


# ---- mission ----------------------------------------------------------------------
class GrabMission(Mission):
    """Mission.on_job_done already sends the robot to the depot when a box is found and
    to the next pickup when one is empty. This changes three things:
      - no box on any branch -> "stop" (stay at the end of the last branch), not home
      - box whose pole couldn't be read -> nearest depot (upper depot on the same side)
      - after a drop, AFTER_DROP = "continue" goes on to the next unchecked branch"""

    def __init__(self, scan_plan, after_drop="home"):
        pickups = ["%s_pickup_%d_end" % (side, n) for side, nums in scan_plan for n in nums]
        Mission.__init__(self, pickups)
        self.after_drop = after_drop
        self.checked = []                  # branches driven down so far, for the log

    def on_job_done(self, polarity=None, box_found=True):
        if self.state == "TO_PICKUP":
            self.checked.append(self.here)
            if not box_found and len(self.pickups) == 1:   # that was the last branch
                self.pickups.pop(0)
                self.state = "DONE"
                return "stop"
            if box_found and polarity is None:   # pole unknown: nearest depot on this side
                self.pickups.pop(0)
                side = self.here.split("_")[0]   # "right" / "left"
                self._go_to(side + "_depot_2_end", "TO_DEPOT")   # depot 2 is nearer the pickups
                return self._leave_towards(self.remaining[0])
        elif self.state == "TO_DEPOT" and self.after_drop == "continue" and self.pickups:
            self.delivered += 1
            self.polarity = None
            self._go_to(self.pickups[0], "TO_PICKUP")
            return self._leave_towards(self.remaining[0])
        return Mission.on_job_done(self, polarity, box_found)


# ---- sensing ----------------------------------------------------------------------
def follow_branch(agv):
    """Follow the branch to the end of the line (no line, or a bar across all four
    sensors, on CONFIRM_READS reads in a row), watching for a box on the way.
    A box counts once the distance is below BOX_DETECT_MM on BOX_CONFIRM_READS reads in
    a row; the robot keeps going to the end either way, so every branch finishes in the
    same place. Returns (found, mm): mm is the reading that confirmed the box, or None."""
    end_count = 0
    box_count = 0
    found_mm = None
    start = ticks_ms()
    while True:
        if found_mm is None:   # stop reading once found: the rest drives like follow_to_end
            mm = agv.read_distance_sensor()
            box_count = box_count + 1 if 0 < mm < BOX_DETECT_MM else 0
            if box_count >= BOX_CONFIRM_READS:
                found_mm = mm
                print("box seen at %d mm, carrying on to the end of the line" % mm)

        r = agv.read_line_sensor()
        if not any(r) or all(r):
            end_count += 1
            if end_count >= CONFIRM_READS:
                agv.stop()
                return found_mm is not None, found_mm
        else:
            end_count = 0
            agv.follow_line(FORWARD_SPEED)
        check_timeout(agv, start, "the end of the branch")
        sleep_ms(10)


# ---- jobs (placeholders until the mechanism and hall sensor are wired in) ---------
def collect(agv, station):
    """Read the box's magnet, then pick it up (placeholder). Returns its polarity as the
    map uses it: "N", "S", or None if no pole was found (-> nearest depot)."""
    pole = agv.detect_pole()
    print("COLLECT at", station, "pole:", pole, "(pick-up is a placeholder)")
    sleep(1)
    return {"north": "N", "south": "S"}.get(pole)


# ---- main loop --------------------------------------------------------------------
def run(agv, mission):
    """Drive the mission from the start box. Same as line_following_test.run, except
    that pickup branches are driven with follow_branch (notes whether there's a box)."""
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < START_FOLLOW_MS:   # get past the box edge
        agv.follow_line(FORWARD_SPEED)
        sleep_ms(5)

    seen = drive_to_junction(agv, lambda: agv.follow_line(FORWARD_SPEED))
    action = mission.on_junction(recheck(agv, mission, seen))
    out_of_pickup = False        # True right after turning around at a pickup branch
    looking_for_depot = False

    while True:
        print("[%s] at %s -> %s" % (mission.state, mission.here, action))
        out_of_branch = False    # this pass turned from a branch onto the main line

        if action == "stop":     # only reached at the start box on the way home
            stop_in_box(agv)
            return
        if action == "straight":
            pass_junction(agv)

        elif action in ("left", "right"):
            latency = DEPOT_CORNER_LATENCY if "depot" in mission.here else CORNER_LATENCY
            turn_at_junction(agv, action, latency)
            out_of_branch = out_of_pickup   # turned out of a pickup branch onto the main line

            if out_of_pickup and mission.state == "TO_PICKUP":
                # Heading on to the next branch: back up a little so its junction is
                # approached from behind (same as line_following_test)
                agv.backward(BACKWARD_SPEED)
                reverse_start = ticks_ms()
                while ticks_diff(ticks_ms(), reverse_start) < BETWEEN_PICKUP_BACK_MS:
                    agv.reverse_follow_line(BACKWARD_SPEED)
                    sleep_ms(5)
                agv.stop()

            if out_of_pickup and mission.state == "TO_DEPOT":
                looking_for_depot = True
        out_of_pickup = False

        if mission.expecting_line_end():
            if mission.state == "TO_PICKUP":
                found, mm = follow_branch(agv)
                mission.on_line_end()
                print("[%s] %s: %s" % (mission.state, mission.here,
                                       "box at %d mm" % mm if found else "empty"))
                polarity = collect(agv, mission.here) if found else None
                action = mission.on_job_done(polarity, box_found=found)
                if action == "stop":
                    print("No box on any branch in SCAN_PLAN: stopping")
                    agv.stop()
                    return
                print("[%s] at %s -> turn around" % (mission.state, mission.here))
                turn_around(agv, back_up=True)
                mission.heading = OPPOSITE[mission.heading]   # facing back the way it came
                out_of_pickup = True

            else:   # depot: drop, reverse out, turn onto the main line
                follow_to_end(agv)   # depot branches: no box check
                mission.on_line_end()
                drop(agv)
                mission.on_job_done()
                print("[%s] at %s -> reverse out of depot" % (mission.state, mission.here))
                reverse_out_of_depot(agv)
                # Nothing is sensed on the way back, so report the junction as the map
                # expects it; the mission gives the turn onto the main line.
                action = mission.on_junction(expected_sides(mission.remaining[0], mission.heading))
                print("[%s] at %s -> %s" % (mission.state, mission.here, action))
                if action not in ("left", "right"):
                    raise RuntimeError("expected a turn out of the depot, got " + action)
                turn_at_junction(agv, action, latency=0)   # axle already over the junction
                out_of_branch = True

        # A turn out of a branch can leave an outer sensor on the next line: back off it
        if out_of_branch:
            back_off_line(agv)

        if looking_for_depot:
            seen = drive_to_depot(agv)
            # Skip the route ahead to the depot junction that drive_to_depot found
            for i, node in enumerate(mission.remaining):
                if "depot" in node and not node.endswith("_end"):
                    mission.remaining = mission.remaining[i:]
                    break
            else:
                raise RuntimeError("No depot junction in route")
            looking_for_depot = False
        else:
            seen = drive_to_junction(agv, lambda: agv.follow_line(FORWARD_SPEED))

        action = mission.on_junction(recheck(agv, mission, seen))


def main():
    agv = AGV()
    mission = GrabMission(SCAN_PLAN, after_drop=AFTER_DROP)
    try:
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
