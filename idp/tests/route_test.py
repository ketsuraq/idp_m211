"""Laptop-only check of the map and state machine (no robot needed):

    python3 idp/tests/route_test.py

Prints the planned routes in plain words and simulates the first test run.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from state_machine import GRAPH, Mission, describe, direction, expected_sides

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

print("\nAll checks passed")
