"""Laptop-only check of the map and state machine (no robot needed):

    python3 idp/tests/route_test.py

Prints the planned routes in plain words and simulates the first test run.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from state_machine import GRAPH, Mission, describe, direction, expected_sides, is_dead_end, route

# Every line in the map must be listed from both ends
for node, lines in GRAPH.items():
    for d, nxt in lines.items():
        if nxt:
            assert nxt in GRAPH, "%s -> unknown node %s" % (node, nxt)
            direction(nxt, node)   # raises if the neighbour doesn't point back

for a, b, heading in [
    ("start", "right_pickup_3_end", "N"),
    ("right_pickup_3_end", "right_depot_2_end", "W"),
    ("right_depot_2_end", "start", "W"),
]:
    print("== %s -> %s" % (a, b))
    print("\n".join(describe(a, b, heading)))

# Simulate the first test with perfect sensors
print("\n== simulated run")
m = Mission(["right_pickup_3_end"])
actions = [m.on_junction("both")]
while not m.finished():
    if m.expecting_line_end():
        actions.append(m.on_line_end())
        actions.append(m.on_job_done(None))
    actions.append(m.on_junction(expected_sides(m.remaining[0], m.heading)))
print(actions)
assert actions == [
    "right", "straight", "left", "straight", "straight", "straight", "straight", "left",
    "collect", "reverse", "left", "straight", "straight", "right",
    "drop", "reverse", "left", "straight", "right", "straight", "stop",
]

# A wrong junction must stop the robot
m = Mission(["right_pickup_3_end"])
try:
    m.on_junction("left")
    raise AssertionError("mismatch not caught")
except Exception as e:
    assert m.state == "ERROR", e


# ---- upper deck and the traffic light ------------------------------------------------
def simulate(m, polarity="S"):
    """Run a mission with perfect sensors. Returns (actions, waits): waits lists the
    junctions where must_wait_for_green() was True, i.e. where the robot waits for green."""
    actions, waits = [m.on_junction("both")], []
    while not m.finished():
        if m.must_wait_for_green():
            waits.append((m.here, m.heading))
        if m.expecting_line_end():
            actions.append(m.on_line_end())
            actions.append(m.on_job_done(polarity))
        actions.append(m.on_junction(expected_sides(m.remaining[0], m.heading)))
    return actions, waits


print("\n== upper deck: start -> upper_left_4 -> right depot -> home")
actions, waits = simulate(Mission(["upper_left_4_end"]))
print(actions, waits)
# Waits once, at the start T going up. Not on the way down (it passes the T heading S, turns E)
# and not when it comes home.
assert waits == [("start", "N")], waits
assert actions[:6] == ["straight"] * 5 + ["left"], actions   # start, box edge, upper 1-3, turn at 4

print("\n== passing through the box (left pickup -> right depot): no wait")
actions, waits = simulate(Mission(["left_pickup_2_end"]))
assert waits == [], waits

print("\n== mid-mission: right depot -> upper_right_2 (arrives at the T from the E, turns N)")
m = Mission(["upper_right_2_end"])
m.here, m.heading = "right_depot_2_end", "S"       # just dropped, reversed out facing S
m.remaining = route(m.here, "upper_right_2_end")[1:]
waits = []
action = m._leave_towards(m.remaining[0])
while m.remaining and not is_dead_end(m.remaining[0]):
    action = m.on_junction(expected_sides(m.remaining[0], m.heading))
    if m.must_wait_for_green():
        waits.append((m.here, action))
print(waits)
assert waits == [("start", "right")], waits        # facing W at the T: turn right to face N

print("\nAll checks passed")
