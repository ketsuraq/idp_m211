"""Laptop-only tests for FirstCompMission's rack progression."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from first_comp import (
    HOME_AT_S, FirstCompMission, is_depot_entry_target, lane_sensor_on_line,
    pickup_node, ramp_transition_plan, upper_turnaround_direction,
)
from state_machine import expected_sides


def arrive_at_branch_end(mission):
    while not mission.expecting_line_end():
        target = mission.remaining[0]
        mission.on_junction(expected_sides(target, mission.heading))
    return mission.remaining[0]


def finish_pickup(mission, box_found=False, polarity=None):
    mission.on_line_end()
    return mission.on_job_done(polarity, box_found)


def finish_depot(mission):
    arrive_at_branch_end(mission)
    mission.on_line_end()
    mission.on_job_done()


mission = FirstCompMission()
assert len(mission.pickups) == 12
assert mission.pickups[0] == pickup_node("upper_left", 1)
assert mission.pickups[6] == pickup_node("upper_right", 1)
assert mission.pickups[-1] == pickup_node("upper_right", 6)
assert lane_sensor_on_line([1, 0, 0, 0], "upper_left")
assert not lane_sensor_on_line([1, 0, 0, 0], "upper_right")
assert lane_sensor_on_line([0, 0, 0, 1], "upper_right")
assert not lane_sensor_on_line([0, 0, 0, 1], "upper_left")
assert is_depot_entry_target("right_depot_2_end")
assert not is_depot_entry_target("right_depot_2")
assert not is_depot_entry_target("right_pickup_2_end")
for side in ("upper_left", "upper_right"):
    assert [upper_turnaround_direction(pickup_node(side, n), box_picked_up=True)
            for n in range(1, 7)] == [
                "right", "right", "right", "left", "left", "left"
            ]
assert upper_turnaround_direction(pickup_node("upper_left", 1)) == "right"
assert upper_turnaround_direction(pickup_node("upper_right", 1)) == "left"
assert ramp_transition_plan("upper_left_1_end", "upper_left_2_end") == (
    "upper_left", 1, "right"
)
assert ramp_transition_plan("upper_right_4_end", "upper_right_5_end") == (
    "upper_right", 4, "left"
)
assert ramp_transition_plan("upper_left_2_end", "upper_left_4_end") is None
assert ramp_transition_plan("upper_left_7_end", "upper_right_1_end") is None

# Empty branches keep the scan in the current rack; the next rack follows branch 6.
for number in range(1, 7):
    assert arrive_at_branch_end(mission) == pickup_node("upper_left", number)
    finish_pickup(mission)
assert mission.pickups[0] == pickup_node("upper_right", 1)

# A successful delivery skips the rest of that rack and starts the next rack.
mission = FirstCompMission()
assert arrive_at_branch_end(mission) == pickup_node("upper_left", 1)
finish_pickup(mission, box_found=True, polarity="N")
assert mission.state == "TO_DEPOT"
assert mission.remaining[-1] == "left_depot_2_end"
finish_depot(mission)
assert mission.delivered == 1
assert "left_depot_2_end" in mission.occupied_depots
assert mission.pickups[0] == pickup_node("upper_right", 1)
assert mission.checked == [pickup_node("upper_left", 1)]

# Same-pole deliveries use the alternate depot across the two scanned racks.
mission = FirstCompMission()
for expected_depot in (
    "right_depot_2_end",
    "right_depot_1_end",
):
    arrive_at_branch_end(mission)
    finish_pickup(mission, box_found=True, polarity="S")
    assert mission.remaining[-1] == expected_depot
    finish_depot(mission)

# When both depots are occupied, the first is reused.
mission = FirstCompMission()
mission.occupied_depots.update(("right_depot_2_end", "right_depot_1_end"))
arrive_at_branch_end(mission)
finish_pickup(mission, box_found=True, polarity="S")
assert mission.remaining[-1] == "right_depot_2_end"

# An unread-pole box left by Grab Test is treated as not taken, so scan the same rack.
mission = FirstCompMission()
assert arrive_at_branch_end(mission) == pickup_node("upper_left", 1)
finish_pickup(mission, box_found=False)
assert mission.pickups[0] == pickup_node("upper_left", 2)

# Nothing taken on the last rack: go home instead of stopping upstairs.
mission = FirstCompMission(clock=lambda: 0)
mission.pickups = [pickup_node("upper_right", 6)]
mission.here = "upper_6"
mission.heading = "E"
mission._go_to(mission.pickups[0], "TO_PICKUP")
mission.remaining = [pickup_node("upper_right", 6)]
assert finish_pickup(mission) == "reverse"
assert mission.state == "TO_HOME"
assert mission.remaining[-1] == "start"

# Time up at an empty branch: go home.
now = [0]
mission = FirstCompMission(clock=lambda: now[0])
arrive_at_branch_end(mission)
now[0] = HOME_AT_S
finish_pickup(mission)
assert mission.state == "TO_HOME"
assert mission.pickups == []

# Time up while picking up a box: deliver it first, then go home.
now = [0]
mission = FirstCompMission(clock=lambda: now[0])
arrive_at_branch_end(mission)
now[0] = HOME_AT_S
finish_pickup(mission, box_found=True, polarity="N")
assert mission.state == "TO_DEPOT"
finish_depot(mission)
assert mission.delivered == 1
assert mission.state == "TO_HOME"

print("FirstCompMission checks passed")
