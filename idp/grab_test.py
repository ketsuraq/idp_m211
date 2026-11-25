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
    back_off_line, check_timeout, drive_to_depot, drive_to_junction, follow_to_end,
    pass_junction,
    recheck, reverse_out_of_depot, stop_in_box, turn_around, turn_at_junction,
    wait_for_green,
)
from state_machine import DEPOT_FOR, FALLBACK_DEPOT, OPPOSITE, Mission, expected_sides
from utime import sleep, sleep_ms, ticks_diff, ticks_ms

# ---- test settings ----------------------------------------------------------------
# Branches to check, side by side, in this order.
#   "left" / "right":             lower pickups 1-6, 1 nearest the corner
#   "upper_left" / "upper_right": upper deck pickups 1-7, 1 nearest the ramp, 7 = the top T.
#                                 Going up the ramp waits in the box for the green light.
# To check more, extend the lists or uncomment more sides.
SCAN_PLAN = [
    ("left", [1, 2, 3, 4, 5, 6]),
    #("right", [1, 2, 3, 4, 5, 6]),
    #("upper_left", [1, 2, 3, 4, 5, 6, 7]),
    #("upper_right", [1, 2, 3, 4, 5, 6, 7]),
]
# After delivering a box: "home" = back to the start box (this test),
# "continue" = carry on with the next unchecked branch.
AFTER_DROP = "home"
# A box is there but the hall sensor found no pole. Which depot it belongs in is unknown:
#   "skip"    = no_pole_skip: leave it, treat the branch as empty, check the next one
#   "nearest" = no_pole_nearest_depot: take it anyway, to the nearest depot on this side
NO_POLE_ACTION = "skip"   # don't carry a box whose pole can't be read

# ---- distance sensor settings -----------------------------------------------------
# While driving down a branch, a reading below this (mm) means there's a box in it.
# The robot doesn't stop for it, it only notes it. Higher = sees the box from further
# away, but is more likely to be set off by something else (e.g. the end wall).

# The reading must be below BOX_DETECT_MM this many times in a row, so a single bad
# reading doesn't count as a box. Higher = fewer false boxes, but a box seen only
# briefly might be missed.
BOX_CONFIRM_READS = 2


# ---- mission ----------------------------------------------------------------------
def pickup_node(side, n):
    """Map node for branch n on a SCAN_PLAN side: lower ones are "left_pickup_3_end",
    upper deck ones are "upper_left_3_end"."""
    if side.startswith("upper_"):
        return "%s_%d_end" % (side, n)
    return "%s_pickup_%d_end" % (side, n)


def side_of(node):
    """"left" or "right" for any pickup node, lower or upper deck."""
    return "left" if "left_" in node else "right"


class GrabMission(Mission):
    """Mission.on_job_done already sends the robot to the depot when a box is found and
    to the next pickup when one is empty. This changes three things:
      - no box on any branch -> "stop" (stay at the end of the last branch), not home
      - box whose pole couldn't be read and was taken (NO_POLE_ACTION = "nearest")
        -> nearest depot (upper depot on the same side). With "skip" the box isn't taken,
        so the mission is told the branch was empty
      - after a drop, AFTER_DROP = "continue" goes on to the next unchecked branch"""

    def __init__(self, scan_plan, after_drop="home"):
        pickups = [pickup_node(side, n) for side, nums in scan_plan for n in nums]
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
                # same side's depot; depot 2 is nearer the pickups
                self._go_to(side_of(self.here) + "_depot_2_end", "TO_DEPOT")
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
        agv.check_stop_button()
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
def no_pole_skip(agv, station):
    """No pole found: leave the box where it is (gripper stays open) and report the branch
    as empty, so the mission moves on to the next one. Returns (polarity, taken)."""
    print("COLLECT at", station, ": no pole, leaving the box")
    return None, False


def no_pole_nearest_depot(agv, station):
    """No pole found: take the box anyway. GrabMission sends a box with no polarity to the
    nearest depot on this side. Returns (polarity, taken)."""
    print("COLLECT at", station, ": no pole, taking it to the nearest depot")
    agv.close_gripper()
    return None, True


NO_POLE_HANDLERS = {"skip": no_pole_skip, "nearest": no_pole_nearest_depot}


def collect(agv, station):
    """At the end of a branch with a box: read the box's magnet. If a pole is found, close
    the gripper on it; if not, NO_POLE_ACTION decides. Returns (polarity, taken):
    polarity is "N", "S" or None (as the map uses it), taken is True if the box is held."""
    pole = {"north": "N", "south": "S"}.get(agv.detect_pole())
    if pole is None:
        return NO_POLE_HANDLERS[NO_POLE_ACTION](agv, station)
    print("COLLECT at", station, "pole:", pole)
    agv.show_polarity(pole)   # red = north, green = south, steady while the box is carried
    agv.close_gripper()
    return pole, True


def drop(agv):
    """At the end of the depot line: open the gripper to let go of the box, then raise the
    arm (it stays raised whenever there's no box). The robot then reverses out
    (reverse_out_of_depot), leaving the box behind."""
    print("DROP: opening gripper")
    agv.open_gripper()
    agv.show_polarity(None)   # box released: red/green off
    print("DROP: raising arm")
    agv.lift()


def back_out_and_lower(agv):
    """After grabbing: back up (line following) for TURN_AROUND_BACK_MS, stop, then lower
    the gripper. Same back-up as turn_around, so the spin afterwards starts from the same
    place as at an empty branch."""
    print("grab: backing out")
    agv.backward(BACKWARD_SPEED)   # no line under the sensors at a pickup end: keep reversing
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < TURN_AROUND_BACK_MS:
        agv.reverse_follow_line(BACKWARD_SPEED)
        sleep_ms(10)
    agv.stop()

def leave_start_box(agv, mission):
    """
    After turning out of the start T, cross the square boundary
    without treating it as a junction.

    Only resume junction detection when:
    - Enough time has passed to cross the boundary.
    - Both middle sensors are on the line.
    - Both outer sensors are clear.
    """

    print("Leaving start box - ignoring boundary")

    agv.forward(FORWARD_SPEED)

    start = ticks_ms()
    clear_count = 0

    while True:
        agv.check_stop_button()
        readings = agv.read_line_sensor()

        elapsed = ticks_diff(ticks_ms(), start)

        # Normal line following, but DO NOT detect junctions
        agv.follow_line(FORWARD_SPEED)

        # Check that we're centred on the line and
        # neither outer sensor is touching the box edge
        centred = readings[1] == 1 and readings[2] == 1
        outer_clear = readings[0] == 0 and readings[3] == 0

        if elapsed >= START_EXIT_IGNORE_MS and centred and outer_clear:
            clear_count += 1
        else:
            clear_count = 0

        if clear_count >= CONFIRM_READS:
            break

        if elapsed > 3000:
            agv.stop()
            raise RuntimeError("Could not clear start box")

        sleep_ms(10)

    print("Start box cleared - normal junction detection enabled")

    # Tell the state machine we passed the box boundary.
    # Both left and right branches exist at this crossing.
    action = mission.on_junction("both")

    if action != "straight":
        agv.stop()
        raise RuntimeError("Expected straight at box edge")

    # Keep driving; don't stop at the boundary.
    agv.forward(FORWARD_SPEED)


def go_up_ramp(agv, action, wait_for_green_signal=True):
    """At the start T, heading for the ramp: end up over the T facing N, optionally wait
    for the traffic light, then drive clear of the T. The IR receiver faces the light here.
      action "straight": already facing N (first trip): drive the wheels onto the T
      action "left"/"right": arrived from E/W: turn N (this leaves the wheels on the T)
    The same stopping spot both ways, so the IR sees the light from the same place."""
    if action == "straight":
        stop_in_box(agv)
    else:
        turn_at_junction(agv, action, CORNER_LATENCY)
    if wait_for_green_signal:
        print("waiting for green")
        wait_for_green(agv)
        print("green: going up the ramp")
    else:
        print("IR wait skipped: going up the ramp")
    pass_junction(agv)   # clear the T's arms; the box edge is then the next junction


# ---- main loop --------------------------------------------------------------------
def run(agv, mission, wait_for_green_signal=True, ramp_pickup_transition=None):
    """Run the mission with the LEDs on: amber flashes the whole time, and every LED is
    switched off at the end, however it ends (finished, stop button or an error).
    Set wait_for_green_signal=False only for a controlled ramp test. An optional
    ramp_pickup_transition handles direct transitions between ramp pickup branches."""
    agv.start_mission_lights()
    try:
        _drive(agv, mission, wait_for_green_signal, ramp_pickup_transition)
    finally:
        agv.stop_mission_lights()


def _drive(agv, mission, wait_for_green_signal=True, ramp_pickup_transition=None):
    """Drive the mission from the start box. Same as line_following_test.run, except
    that pickup branches are driven with follow_branch (notes whether there's a box)."""
    agv.lift()           # no box yet: arm raised
    agv.open_gripper()   # and gripper open, ready to close on the first box
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < START_FOLLOW_MS:   # get past the box edge
        agv.check_stop_button()
        agv.follow_line(FORWARD_SPEED)
        sleep_ms(5)

    seen = drive_to_junction(agv, lambda: agv.follow_line(FORWARD_SPEED))
    action = mission.on_junction(recheck(agv, mission, seen))
    out_of_pickup = False        # True right after turning around at a pickup branch
    looking_for_depot = False
    ramp_pickup_ready = False

    while True:
        agv.check_stop_button()

        out_of_branch = False    # this pass turned from a branch onto the main line
        if ramp_pickup_ready:
            ramp_pickup_ready = False
            action = None
        else:
            print("[%s] at %s -> %s" % (mission.state, mission.here, action))

        if action == "stop":     # only reached at the start box on the way home
            stop_in_box(agv)
            return
        if mission.must_wait_for_green():
            # At the start T, face N and pass the box_top_edge as a normal junction.
            go_up_ramp(agv, action, wait_for_green_signal)
        elif action == "straight":
            pass_junction(agv)

        elif action in ("left", "right"):
            latency = DEPOT_CORNER_LATENCY if "depot" in mission.here else CORNER_LATENCY
            turn_at_junction(agv, action, latency)

            # Special handling for leaving the start square
            if mission.here == "start":
                leave_start_box(agv, mission)

            out_of_branch = out_of_pickup   # turned out of a pickup branch onto the main line

            if out_of_pickup and mission.state == "TO_PICKUP":
                # Heading on to the next branch: back up a little so its junction is
                # approached from behind (same as line_following_test)
                agv.backward(BACKWARD_SPEED)
                reverse_start = ticks_ms()
                while ticks_diff(ticks_ms(), reverse_start) < BETWEEN_PICKUP_BACK_MS:
                    agv.check_stop_button()

                    agv.reverse_follow_line(BACKWARD_SPEED)
                    sleep_ms(5)
                agv.stop()

            # drive_to_depot finds the depot by the first long gap between junctions. From
            # the upper deck the ramp is a long gap too, so there the map's junction count
            # is used instead (the normal drive_to_junction below).
            if out_of_pickup and mission.state == "TO_DEPOT" and not mission.here.startswith("upper_"):
                looking_for_depot = True
        out_of_pickup = False

        if mission.expecting_line_end():
            if mission.state == "TO_PICKUP":
                found, mm = follow_branch(agv)
                mission.on_line_end()
                print("[%s] %s: %s" % (mission.state, mission.here,
                                       "box at %d mm" % mm if found else "empty"))
                polarity, taken = collect(agv, mission.here) if found else (None, False)
                action = mission.on_job_done(polarity, box_found=taken)
                if action == "stop":
                    print("No box taken on any branch in SCAN_PLAN: stopping")
                    agv.stop()
                    return
                transitioned = False
                if not taken and ramp_pickup_transition is not None:
                    transitioned = ramp_pickup_transition(agv, mission)
                if transitioned:
                    ramp_pickup_ready = True
                    out_of_pickup = False
                else:
                    print("[%s] at %s -> turn around" % (mission.state, mission.here))
                    if taken:   # holding the box: back out, lower, then spin
                        back_out_and_lower(agv)
                        turn_around(agv, back_up=False)
                    else:
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
        elif ramp_pickup_ready:
            continue
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
