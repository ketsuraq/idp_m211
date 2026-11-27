"""First competition mission using Grab Test's driving and pickup routines.

Scan upper left, then upper right, branches 1-6 in order. After the first successful
delivery from a rack, skip its remaining branches and move to the next rack. A box with
an unreadable pole is left in place, following Grab Test's default policy. The robot
waits for the IR green signal before going up the ramp (WAIT_FOR_GREEN_SIGNAL). It goes
home when nothing is left to scan, or after HOME_AT_S (delivering any box it is carrying
first). Depot occupancy is tracked from drops made during this run, assuming the depots
start empty. For each pole, use depot 2 first, depot 1 second, and depot 2 again if both
have already been used.

Lower floor: grab_test's driving, unchanged.

Upper deck (tuned with upper_test.py, which now just runs this file without the lower
racks). run() swaps these in for grab_test's own moves, only on the upper racks:
  - Empty lanes keep the timed ramp_pickup_transition to the next lane on the same rack.
  - upper_turn_around / upper_back_out: after a pickup, branches 1-3 spin right and
    branches 4-6 spin left. Empty-lane fallback turnarounds use the rack's prior direction.
  - upper_spine_turn: turning out of a lane onto the spine, heading down to the ramp, with
    its own blind time. skip_lane_passed_in_turn then counts the lane the turn carried the
    sensors past.
  - finding a lane on the spine (drive_to_lane): EITHER outer sensor on a line counts, and
    steering never pauses (follow_line holds each correction for 100 ms without reading
    the sensors, long enough for a lane to pass under them unseen).
All the timed moves on the upper deck are in UPPER_SIDE, separately for each rack.
"""

from constants import (
    BACKWARD_SPEED, CARRYING_RIGHT_PRETURN_MS, CARRYING_RIGHT_TURN_BLIND_MS,
    CONFIRM_READS, CORNER_LATENCY, FORWARD_SPEED, TURN_ALIGN_SETTLE_MS, TURN_AROUND_SPEED,
    TURN_AROUND_TIMEOUT_MS, TURN_SPEED, TURN_TIMEOUT_MS,
)
from state_machine import (
    DEPOT_FOR, FALLBACK_DEPOT, Mission, MissionError, UPPER_CROSSINGS, expected_sides,
)


WAIT_FOR_GREEN_SIGNAL = True
# Seconds after the start to stop scanning and head home (the run is 300 s). Only checked
# at the end of a branch and after a drop, so leave enough margin to get back.
HOME_AT_S = 240

# ---- timed moves on the upper deck, per rack (all in ms) -------------------------------
# The robot turns and reverses by different amounts in each direction, so each rack has
# its own values. Tuned on the table with upper_test.py.
UPPER_SIDE = {
    "upper_left": {
        # moving on from an EMPTY lane to the next lane of the same rack
        "backout_ms": 1300,           # reverse out of the lane until the axle is on the spine
        "pivot_ms": 900,              # 90 degree pivot to face up the spine (pivots right)
        "after_pivot_back_ms": 650,   # reverse after the pivot, before driving to the next lane
        "turn_in_overshoot_ms": 50,   # keep turning this long after finding the next lane
        # the 180 at the end of a lane (after a grab, or at the last lane of the rack)
        "turn_around_back_ms": 500,   # back up before the spin
        "turn_around_blind_ms": 1100, # blind part of the spin: RAISE if it stops on another line
        "turn_around_direction": "right",   # prior direction for empty-lane fallback
        # turning out of a lane onto the spine, heading down to the ramp (turns right)
        "spine_turn_blind_ms": 662,   # blind part of the turn (was the shared TURN_TIME)
    },
    "upper_right": {
        "backout_ms": 1300,
        "pivot_ms": 1100,             # pivots left
        "after_pivot_back_ms": 500,
        "turn_in_overshoot_ms": 50,
        "turn_around_back_ms": 500,
        "turn_around_blind_ms": 1100,
        "turn_around_direction": "left",
        "spine_turn_blind_ms": 862,   # turns left onto the spine
    },
}


# A lane on the spine counts once an outer sensor sees a line on this many reads in a row
# (reads are LANE_READ_MS apart). Higher risks missing a lane crossed quickly.
LANE_CONFIRM_READS = 2
LANE_READ_MS = 5

ALTERNATE_DEPOT = {
    "left_depot_2_end": "left_depot_1_end",
    "right_depot_2_end": "right_depot_1_end",
}

    # ("left", tuple(range(1, 7))),
    # ("right", tuple(range(1, 7))),

RACKS = (
    ("left", tuple(range(1, 7))),
    ("right", tuple(range(1, 7))),
    ("upper_left", tuple(range(1, UPPER_CROSSINGS + 1))),
    ("upper_right", tuple(range(1, UPPER_CROSSINGS + 1))),
)


def pickup_node(side, number):
    """Return the map node at the end of a pickup branch."""
    if side.startswith("upper_"):
        return "%s_%d_end" % (side, number)
    return "%s_pickup_%d_end" % (side, number)


def ramp_transition_plan(current_pickup, next_pickup):
    """Return (side, branch, turn) only for the next branch on the same upper rack."""
    for side, turn in (("upper_left", "right"), ("upper_right", "left")):
        prefix = side + "_"
        suffix = "_end"
        if current_pickup.startswith(prefix) and current_pickup.endswith(suffix):
            number_text = current_pickup[len(prefix):-len(suffix)]
            if not number_text.isdigit():
                return None
            number = int(number_text)
            if next_pickup == pickup_node(side, number + 1):
                return side, number, turn
    return None


def rack_side(node):
    """"upper_left" / "upper_right" for an upper lane end like "upper_left_3_end", else None."""
    side = (node or "").rsplit("_", 2)[0]
    return side if side in UPPER_SIDE else None


def is_depot_entry_target(node):
    """Return whether the next route node is a depot branch end."""
    return bool(node and "_depot_" in node and node.endswith("_end"))


def upper_turnaround_direction(node, box_picked_up=False):
    """Return the pickup-specific direction or prior rack direction for an empty lane."""
    parts = (node or "").rsplit("_", 2)
    if len(parts) != 3 or parts[0] not in UPPER_SIDE or parts[2] != "end":
        raise MissionError("not an upper pickup branch end: " + str(node))
    if not parts[1].isdigit():
        raise MissionError("invalid upper pickup branch number: " + str(node))
    branch_number = int(parts[1])
    if branch_number < 1 or branch_number > UPPER_CROSSINGS:
        raise MissionError("upper pickup branch out of range: " + str(node))
    if not box_picked_up:
        return UPPER_SIDE[parts[0]]["turn_around_direction"]
    return "right" if branch_number <= 3 else "left"


class FirstCompMission(Mission):
    """Visit each rack in order, delivering at most one box from each rack."""

    def __init__(self, clock=None):
        """clock() returns seconds since the start; by default it starts counting now."""
        if clock is None:
            from time import time
            started = time()
            clock = lambda: time() - started
        self.clock = clock
        pickups = []
        self._rack_by_pickup = {}
        for rack_index, (side, numbers) in enumerate(RACKS):
            for number in numbers:
                node = pickup_node(side, number)
                pickups.append(node)
                self._rack_by_pickup[node] = rack_index

        Mission.__init__(self, pickups)
        self.checked = []
        self._delivering_from_rack = None
        self._delivering_to = None
        self.occupied_depots = set()

    def on_job_done(self, polarity=None, box_found=True):
        if self.state == "TO_PICKUP":
            self.checked.append(self.here)
            rack_index = self._rack_by_pickup[self.here]
            self.pickups.pop(0)

            if box_found:
                self.polarity = polarity
                self._delivering_from_rack = rack_index
                first_depot = DEPOT_FOR.get(polarity, FALLBACK_DEPOT)
                alternate_depot = ALTERNATE_DEPOT[first_depot]
                if first_depot not in self.occupied_depots:
                    self._delivering_to = first_depot
                elif alternate_depot not in self.occupied_depots:
                    self._delivering_to = alternate_depot
                else:
                    self._delivering_to = first_depot
                self._go_to(self._delivering_to, "TO_DEPOT")
            else:
                self._next_pickup_or_home()

        elif self.state == "TO_DEPOT":
            if self._delivering_from_rack is None:
                self.state = "ERROR"
                raise MissionError("delivered box has no source rack")
            if self._delivering_to is None:
                self.state = "ERROR"
                raise MissionError("delivered box has no destination depot")

            self.delivered += 1
            self.occupied_depots.add(self._delivering_to)
            self.polarity = None
            rack_index = self._delivering_from_rack
            self._delivering_from_rack = None
            self._delivering_to = None
            self.pickups = [
                node for node in self.pickups
                if self._rack_by_pickup[node] != rack_index
            ]

            self._next_pickup_or_home()

        return self._leave_towards(self.remaining[0])

    def _next_pickup_or_home(self):
        """Head for the next branch, or home if none are left or time is up."""
        if self.clock() >= HOME_AT_S:
            print("time's up: going home")
            self.pickups = []
        if self.pickups:
            self._go_to(self.pickups[0], "TO_PICKUP")
        else:
            self._go_to("start", "TO_HOME")


# ---- upper deck: steering without pauses ---------------------------------------------
def steer_forward(agv, r):
    """One forward steering step from readings r. Unlike follow_line it never sleeps, so
    the caller keeps reading the sensors."""
    if r[1] and not r[2]:
        agv.correct("left", FORWARD_SPEED)
    elif r[2] and not r[1]:
        agv.correct("right", FORWARD_SPEED)
    else:                              # centred, or no line under the middle: straight on
        agv.forward(FORWARD_SPEED)


def steer_backward(agv, r):
    """One reversing steering step (same sides as reverse_follow_line), never sleeps."""
    if r[1] and not r[2]:
        agv.reverse_correct("left", BACKWARD_SPEED)
    elif r[2] and not r[1]:
        agv.reverse_correct("right", BACKWARD_SPEED)
    else:
        agv.backward(BACKWARD_SPEED)


def outer_on_line(r):
    return bool(r[0] or r[3])


def lane_sensor_on_line(r, side=None):
    if side == "upper_left":
        return bool(r[0])
    if side == "upper_right":
        return bool(r[3])
    if side is None:
        return outer_on_line(r)
    raise ValueError("unknown rack side: " + str(side))


# ---- upper deck: lane finding and reversing ---------------------------------------------
def drive_to_lane(agv, side=None):
    """Drive up the spine to the next lane, using that rack's outer sensor when specified.
    First clear the lane just left; then confirm the next lane under the sensors."""
    from line_following_test import check_timeout
    from utime import sleep_ms, ticks_ms

    start = ticks_ms()
    clear = 0
    while clear < LANE_CONFIRM_READS:          # 1. clear of any lane already underneath
        agv.check_stop_button()
        r = agv.read_line_sensor()
        clear = clear + 1 if not lane_sensor_on_line(r, side) else 0
        steer_forward(agv, r)
        check_timeout(agv, start, "the outer sensors to clear (upper spine)")
        sleep_ms(LANE_READ_MS)

    seen = 0
    while seen < LANE_CONFIRM_READS:           # 2. the next lane
        agv.check_stop_button()
        r = agv.read_line_sensor()
        if lane_sensor_on_line(r, side):
            seen += 1                          # keep the current motor command over the lane
        else:
            seen = 0
            steer_forward(agv, r)
        check_timeout(agv, start, "the next upper lane")
        sleep_ms(LANE_READ_MS)
    print("lane found:", r)


def reverse_steering_for(agv, milliseconds):
    """Reverse (steering on the spine, never pausing) for exactly milliseconds, then stop.
    Doesn't stop at lanes."""
    from utime import sleep_ms, ticks_diff, ticks_ms

    agv.backward(BACKWARD_SPEED)
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < milliseconds:
        agv.check_stop_button()
        steer_backward(agv, agv.read_line_sensor())
        sleep_ms(LANE_READ_MS)
    agv.stop()


def reverse_for(agv, milliseconds):
    """Reverse with line following for milliseconds, then stop."""
    from utime import sleep_ms, ticks_diff, ticks_ms

    agv.backward(BACKWARD_SPEED)
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < milliseconds:
        agv.check_stop_button()
        agv.reverse_follow_line(BACKWARD_SPEED)
        sleep_ms(10)
    agv.stop()


# ---- upper deck: the move between lanes -------------------------------------------------
def ramp_pickup_transition(agv, mission):
    """Move directly from an empty upper lane to the next lane on the same rack. Returns
    False (nothing done) when that doesn't apply, so the driver turns around as usual."""
    if not mission.pickups:
        return False
    plan = ramp_transition_plan(mission.here, mission.pickups[0])
    if plan is None:
        return False

    side, branch_number, turn_direction = plan
    junction = "upper_%d" % branch_number
    next_junction = "upper_%d" % (branch_number + 1)
    if (
        len(mission.remaining) < 2
        or mission.remaining[0] != junction
        or mission.remaining[1] != next_junction
    ):
        return False

    from line_following_test import turn_at_junction
    from utime import sleep_ms, ticks_diff, ticks_ms

    t = UPPER_SIDE[side]

    # 1. Back out of the lane until the axle is on the spine (timed)
    print("ramp (%s): reverse-following for %d ms" % (side, t["backout_ms"]))
    reverse_for(agv, t["backout_ms"])

    # Timed, so nothing is sensed: report this junction as the map expects it
    action = mission.on_junction(expected_sides(junction, mission.heading))
    if action != turn_direction:
        raise RuntimeError("unexpected turn at upper rack junction: " + action)

    # 2. Pivot 90 degrees to face up the spine (timed)
    print("ramp (%s): pivoting %s for %d ms" % (side, action, t["pivot_ms"]))
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < t["pivot_ms"]:
        agv.check_stop_button()
        agv.turn(action, TURN_SPEED)
        sleep_ms(10)
    agv.stop()

    # 3. Back up for exactly after_pivot_back_ms. On the table the sensors end up
    #    BETWEEN the lane just left and the next lane, so no lane counting here.
    print("ramp (%s): reversing for %d ms" % (side, t["after_pivot_back_ms"]))
    reverse_steering_for(agv, t["after_pivot_back_ms"])

    # 4. Forward to the first lane it meets (the next lane), then turn in
    print("ramp: driving forward to", next_junction)
    drive_to_lane(agv, side)
    action = mission.on_junction(expected_sides(next_junction, mission.heading))
    if action not in ("left", "right"):
        raise RuntimeError("expected a turn into the next pickup, got " + action)
    turn_at_junction(
        agv,
        action,
        settle_ms=TURN_ALIGN_SETTLE_MS + t["turn_in_overshoot_ms"],
    )
    return True


# ---- upper deck: the 180 at the end of a lane -------------------------------------------
def upper_back_out(agv, side):
    """grab_test.back_out_and_lower with this rack's back-up time: back up (line
    following), then stop. (Like the original, it doesn't lower the gripper.)"""
    from utime import sleep_ms, ticks_diff, ticks_ms

    ms = UPPER_SIDE[side]["turn_around_back_ms"]
    print("grab (%s): backing out for %d ms" % (side, ms))
    agv.backward(BACKWARD_SPEED)
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < ms:
        agv.reverse_follow_line(BACKWARD_SPEED)
        sleep_ms(10)
    agv.stop()


def upper_turn_around(agv, side, direction, back_up=True):
    """line_following_test.turn_around with this rack's back-up and blind time.
    Back up (unless already done), spin blind, then keep spinning until the middle sensors
    leave the line and find it again."""
    from utime import sleep_ms, ticks_diff, ticks_ms

    t = UPPER_SIDE[side]
    if back_up:
        print("turn around (%s): backing up for %d ms" % (side, t["turn_around_back_ms"]))
        agv.backward(BACKWARD_SPEED)
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < t["turn_around_back_ms"]:
            agv.check_stop_button()
            agv.reverse_follow_line(BACKWARD_SPEED)
            sleep_ms(10)
    print("turn around (%s): spinning %s (blind %d ms)"
          % (side, direction, t["turn_around_blind_ms"]))
    agv.turn(direction, TURN_AROUND_SPEED)
    agv.interruptible_sleep(t["turn_around_blind_ms"])
    start = ticks_ms()
    try:
        # No sleep: spinning fast, the middle sensors are only both on a line for a moment
        while agv.centred_on_line():
            agv.check_stop_button()
            if ticks_diff(ticks_ms(), start) > TURN_AROUND_TIMEOUT_MS:
                raise RuntimeError("Turn around timeout (never left the line)")
        while not agv.centred_on_line():
            agv.check_stop_button()
            if ticks_diff(ticks_ms(), start) > TURN_AROUND_TIMEOUT_MS:
                raise RuntimeError("Turn around timeout (never found the line)")
        print("turn around: done, back on the line")
    finally:
        agv.stop()


# ---- upper deck: turning out of a lane onto the spine, heading down to the ramp ---------
SPINE_TURN_SIDE = {"right": "upper_left", "left": "upper_right"}   # turn direction -> rack


def spine_turn_side(mission, direction):
    """The rack ("upper_left"/"upper_right") if this turn is out of an upper lane onto the
    spine heading down (S) to the ramp, else None. Called with the mission already at the
    spine junction ("upper_3") and facing S, as the driver leaves it before turning."""
    here = mission.here or ""
    if here.startswith("upper_") and here[len("upper_"):].isdigit() and mission.heading == "S":
        return SPINE_TURN_SIDE.get(direction)
    return None


def upper_spine_turn(agv, direction, latency, blind_ms, settle_ms):
    """line_following_test.turn_at_junction with its own blind time: walk forward `latency`
    seconds so the wheels are over the junction, turn blind for blind_ms, keep turning until
    both middle sensors confirm the spine, then settle and stop."""
    from utime import sleep_ms, ticks_diff, ticks_ms

    # 1. Walk forward (line following) to put the wheel axle over the junction
    if latency > 0:
        agv.forward(FORWARD_SPEED)
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < latency * 1000:
        agv.check_stop_button()
        agv.follow_line(FORWARD_SPEED)
        sleep_ms(10)
    # 2. Blind turn to get the sensors off the lane line
    print("spine turn: %s, blind %d ms" % (direction, blind_ms))
    agv.turn(direction, TURN_SPEED)
    agv.interruptible_sleep(blind_ms)
    # 3. Keep turning until both middle sensors confirm the spine
    start = ticks_ms()
    confirm_count = 0
    while True:
        agv.check_stop_button()
        readings = agv.read_line_sensor()
        if readings[1] and readings[2]:
            confirm_count += 1
            if confirm_count >= CONFIRM_READS:
                break
        else:
            confirm_count = 0
        if ticks_diff(ticks_ms(), start) > TURN_TIMEOUT_MS:
            raise RuntimeError("Turn timed out without finding the line")
        sleep_ms(10)
    # 4. Keep turning briefly after confirmation, then stop
    agv.interruptible_sleep(settle_ms)
    agv.stop()


def skip_lane_passed_in_turn(agv, mission):
    """Call straight after turning at spine junction upper_N onto the spine heading S
    (towards the ramp). The turn carries the sensors past lane N-1 on the table, so count
    it now: the robot then waits for one lane fewer before the ramp.
    Not if an outer sensor is on a line after the turn: then the sensors stopped ON that
    lane, and the driver's back_off_line reverses off it so it is counted normally."""
    here = mission.here or ""
    if not (here.startswith("upper_") and here[len("upper_"):].isdigit()):
        return                              # not a spine junction ("upper_3")
    n = int(here[len("upper_"):])
    lane_below = "upper_%d" % (n - 1)
    if mission.heading != "S" or not mission.remaining or mission.remaining[0] != lane_below:
        return                              # not heading down past another lane
    if outer_on_line(agv.read_line_sensor()):
        print("upper: sensors on", lane_below, "after the turn - counting it normally")
        return
    mission.on_junction(expected_sides(lane_below, mission.heading))
    print("upper: turn carried the sensors past", lane_below, "- counted it")


# ---- run -------------------------------------------------------------------------------
def run(agv, mission):
    """grab_test's driver with upper-specific turnarounds and spine transitions.
    Empty lanes use the timed ramp transition; pickups use branch-specific 180s."""
    import grab_test

    original_turn_around = grab_test.turn_around
    original_turn_at_junction = grab_test.turn_at_junction
    original_back_out_and_lower = grab_test.back_out_and_lower

    def back_out_and_lower(agv):
        side = rack_side(mission.here)
        if side:
            upper_back_out(agv, side)
        else:
            original_back_out_and_lower(agv)         # anywhere else: unchanged

    def turn_at_junction(agv, direction, latency=CORNER_LATENCY, settle_ms=TURN_ALIGN_SETTLE_MS):
        side = spine_turn_side(mission, direction)
        if side:   # out of an upper lane onto the spine: this rack's blind time
            upper_spine_turn(agv, direction, latency,
                             UPPER_SIDE[side]["spine_turn_blind_ms"], settle_ms)
        elif (
            direction == "right"
            and mission.state == "TO_DEPOT"
            and not (mission.here or "").startswith("upper_")
            and not (
                mission.remaining
                and is_depot_entry_target(mission.remaining[0])
            )
        ):
            original_turn_at_junction(
                agv,
                direction,
                latency,
                settle_ms,
                pre_turn_ms=CARRYING_RIGHT_PRETURN_MS,
                blind_time_ms=CARRYING_RIGHT_TURN_BLIND_MS,
            )
        else:      # any other turn: unchanged
            original_turn_at_junction(agv, direction, latency, settle_ms)
        skip_lane_passed_in_turn(agv, mission)

    def turn_around(agv, back_up=True):
        side = rack_side(mission.here)
        if side:
            direction = upper_turnaround_direction(
                mission.here, box_picked_up=not back_up
            )
            upper_turn_around(agv, side, direction, back_up)
        else:
            original_turn_around(agv, back_up)       # anywhere else: unchanged

    grab_test.back_out_and_lower = back_out_and_lower
    grab_test.turn_at_junction = turn_at_junction
    grab_test.turn_around = turn_around
    try:
        return grab_test.run(
            agv,
            mission,
            wait_for_green_signal=WAIT_FOR_GREEN_SIGNAL,
            ramp_pickup_transition=ramp_pickup_transition,
        )
    finally:
        grab_test.turn_around = original_turn_around
        grab_test.turn_at_junction = original_turn_at_junction
        grab_test.back_out_and_lower = original_back_out_and_lower


