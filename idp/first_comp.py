"""First competition mission using Grab Test's driving and pickup routines.

Scan racks in order: lower left, lower right, upper left, upper right. Scan branches
in ascending order within each rack. After the first successful delivery from a rack,
skip its remaining branches and move to the next rack. A box with an unreadable pole is
left in place, following Grab Test's default policy. The IR green wait is disabled for
this test; only run it when the ramp is clear. Depot occupancy is tracked from drops made
during this run, assuming the depots start empty. For each pole, use depot 2 first, depot
1 second, and depot 2 again if both have already been used.
"""

from constants import (
    BACKWARD_SPEED, FORWARD_SPEED, TURN_ALIGN_SETTLE_MS, TURN_AROUND_BACK_MS, TURN_SPEED,
)
from state_machine import (
    DEPOT_FOR, FALLBACK_DEPOT, Mission, MissionError, UPPER_CROSSINGS, expected_sides,
)


WAIT_FOR_GREEN_SIGNAL = True
RAMP_BACKOUT_MS = 1200
RAMP_REVERSE_TURN_MS = 1400
RAMP_TURN_OVERSHOOT_MS = 50

ALTERNATE_DEPOT = {
    "left_depot_2_end": "left_depot_1_end",
    "right_depot_2_end": "right_depot_1_end",
}

RACKS = (
    ("left", tuple(range(1, 7))),
    ("right", tuple(range(1, 7))),
    ("upper_left", tuple(range(1, UPPER_CROSSINGS + 2))),
    ("upper_right", tuple(range(1, UPPER_CROSSINGS + 2))),
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


class FirstCompMission(Mission):
    """Visit each rack in order, delivering at most one box from each rack."""

    def __init__(self):
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
            elif self.pickups:
                self._go_to(self.pickups[0], "TO_PICKUP")
            else:
                self.state = "DONE"
                return "stop"

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

            if self.pickups:
                self._go_to(self.pickups[0], "TO_PICKUP")
            else:
                self._go_to("start", "TO_HOME")

        return self._leave_towards(self.remaining[0])


def ramp_pickup_transition(agv, mission):
    """Move directly from an empty upper branch to the next branch on the same rack."""
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

    from line_following_test import drive_to_junction, recheck, turn_at_junction
    from utime import sleep_ms, ticks_diff, ticks_ms

    def reverse_follow_for(milliseconds):
        agv.backward(BACKWARD_SPEED)
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < milliseconds:
            agv.check_stop_button()
            agv.reverse_follow_line(BACKWARD_SPEED)
            sleep_ms(10)
        agv.stop()

    print("ramp: reverse-following for", RAMP_BACKOUT_MS, "ms")
    reverse_follow_for(RAMP_BACKOUT_MS)

    # The timed backout assumes the robot has reached this spine junction.
    action = mission.on_junction(expected_sides(junction, mission.heading))
    if action != turn_direction:
        raise RuntimeError("unexpected turn at upper rack junction: " + action)

    print("ramp: pivoting", action, "for", RAMP_REVERSE_TURN_MS, "ms")
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < RAMP_REVERSE_TURN_MS:
        agv.check_stop_button()
        agv.turn(action, TURN_SPEED)
        sleep_ms(10)
    agv.stop()

    print("ramp: reversing for", TURN_AROUND_BACK_MS, "ms")
    reverse_follow_for(int(TURN_AROUND_BACK_MS))

    print("ramp: driving forward to", next_junction)
    agv.forward(FORWARD_SPEED)
    seen = drive_to_junction(agv, lambda: agv.follow_line(FORWARD_SPEED))
    action = mission.on_junction(recheck(agv, mission, seen))
    if action not in ("left", "right"):
        raise RuntimeError("expected a turn into the next pickup, got " + action)
    turn_at_junction(
        agv,
        action,
        settle_ms=TURN_ALIGN_SETTLE_MS + RAMP_TURN_OVERSHOOT_MS,
    )
    return True


def run(agv, mission):
    """Use Grab Test's driver with First Comp's ramp-specific pickup transition."""
    from grab_test import run as grab_test_run

    return grab_test_run(
        agv,
        mission,
        wait_for_green_signal=WAIT_FOR_GREEN_SIGNAL,
        ramp_pickup_transition=ramp_pickup_transition,
    )

