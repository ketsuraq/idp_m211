"""First competition mission using Grab Test's driving and pickup routines.

Scan racks in order: lower left, lower right, upper left, upper right. Scan branches
in ascending order within each rack. After the first successful delivery from a rack,
skip its remaining branches and move to the next rack. A box with an unreadable pole is
left in place, following Grab Test's default policy. The IR green wait is disabled for
this test; only run it when the ramp is clear. Depot occupancy is tracked from drops made
during this run, assuming the depots start empty. For each pole, use depot 2 first, depot
1 second, and depot 2 again if both have already been used.
"""

from constants import BACKWARD_SPEED, CONFIRM_READS, FORWARD_SPEED, TURN_SPEED, TURN_TIME
from state_machine import (
    DEPOT_FOR, FALLBACK_DEPOT, Mission, MissionError, UPPER_CROSSINGS, expected_sides,
)


WAIT_FOR_GREEN_SIGNAL = False

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

    from utime import sleep_ms

    print("ramp: reverse-following to", junction)
    agv.backward(BACKWARD_SPEED)
    junction_count = 0
    while junction_count < CONFIRM_READS:
        agv.check_stop_button()
        readings = agv.read_line_sensor()
        if readings[0] or readings[3] or readings == [1, 1, 1, 1]:
            junction_count += 1
        else:
            junction_count = 0
            agv.reverse_follow_line(BACKWARD_SPEED)
        sleep_ms(10)
    agv.stop()

    # Accept any detected junction pattern; the route map supplies the logical sides.
    mission.on_junction(expected_sides(junction, mission.heading))

    print("ramp: turning 45 degrees", turn_direction)
    agv.turn(turn_direction, TURN_SPEED)
    agv.interruptible_sleep(int(TURN_TIME * 500))
    agv.stop()

    print("ramp: reacquiring line with middle sensors")
    agv.forward(FORWARD_SPEED)
    while agv.centred_on_line():
        agv.check_stop_button()
        agv.forward(FORWARD_SPEED)
        sleep_ms(10)

    centred_count = 0
    while centred_count < CONFIRM_READS:
        agv.check_stop_button()
        if agv.centred_on_line():
            centred_count += 1
        else:
            centred_count = 0
            agv.forward(FORWARD_SPEED)
        sleep_ms(10)
    agv.stop()

    # The diagonal enters the next pickup branch without traversing its spine junction.
    mission.on_junction(expected_sides(next_junction, mission.heading))
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
