"""Mission state machine: decides where the robot goes and what it does at each junction.

No hardware imports, so it can be run and tested on a laptop (tests/route_test.py).
See docs/navigation.md for the concepts.

Compass convention (stand at the start box facing the ramp):
    N = towards the ramp / racks    S = towards the front edge of the table
    E = right                       W = left
"""

DIRS = ["N", "E", "S", "W"]          # clockwise
OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}
STOP_ON_MISMATCH = True              # testing: stop if a junction doesn't look like the map says

# ---------------------------------------------------------------------------
# The map. Each junction lists the lines leaving it, by compass direction:
#   "E": "right_corner"   following the line east leads to right_corner
#   "N": None             a line is there (the sensors will see it) but we never take it
# A node with only one line is a dead end (the end of a branch).
# Only the right side is written out; the left side is its mirror image (see _mirror).
# Depot/pickup 1 is the one nearest the corner.
# ---------------------------------------------------------------------------
RIGHT_SIDE = {
    "right_box_edge":   {"W": "start", "E": "right_corner", "N": None, "S": None},  # box edge crosses the line
    "right_corner":     {"W": "right_box_edge", "N": "right_depot_1"},
    "right_depot_1":    {"S": "right_corner", "N": "right_depot_2", "W": "right_depot_1_end"},
    "right_depot_2":    {"S": "right_depot_1", "N": "right_pickup_1", "W": "right_depot_2_end"},
    "right_pickup_1":   {"S": "right_depot_2", "N": "right_pickup_2", "W": "right_pickup_1_end"},
    "right_pickup_2":   {"S": "right_pickup_1", "N": "right_pickup_3", "W": "right_pickup_2_end"},
    "right_pickup_3":   {"S": "right_pickup_2", "N": "right_pickup_4", "W": "right_pickup_3_end"},
    "right_pickup_4":   {"S": "right_pickup_3", "N": "right_pickup_5", "W": "right_pickup_4_end"},
    "right_pickup_5":   {"S": "right_pickup_4", "N": "right_pickup_6", "W": "right_pickup_5_end"},
    "right_pickup_6":   {"S": "right_pickup_5", "N": "right_end", "W": "right_pickup_6_end"},
    "right_end":        {"S": "right_pickup_6"},
    "right_depot_1_end":  {"E": "right_depot_1"},
    "right_depot_2_end":  {"E": "right_depot_2"},
    "right_pickup_1_end": {"E": "right_pickup_1"},
    "right_pickup_2_end": {"E": "right_pickup_2"},
    "right_pickup_3_end": {"E": "right_pickup_3"},
    "right_pickup_4_end": {"E": "right_pickup_4"},
    "right_pickup_5_end": {"E": "right_pickup_5"},
    "right_pickup_6_end": {"E": "right_pickup_6"},
}

CENTRE = {
    # T inside the start box. The robot starts below it (no line there) facing N.
    "start":          {"E": "right_box_edge", "W": "left_box_edge", "N": "box_top_edge"},
    "box_top_edge":   {"S": "start", "N": "ramp_bottom", "E": None, "W": None},
    "ramp_bottom":    {"S": "box_top_edge"},  # TODO: extend with the ramp / upper deck
}


def _mirror(side):
    """Left side = right side with E<->W and right_<->left_ swapped."""
    flip = {"E": "W", "W": "E", "N": "N", "S": "S"}

    def rename(n):
        return n.replace("right_", "left_") if n else n

    return {rename(node): {flip[d]: rename(n) for d, n in lines.items()}
            for node, lines in side.items()}


GRAPH = {}
GRAPH.update(CENTRE)
GRAPH.update(RIGHT_SIDE)
GRAPH.update(_mirror(RIGHT_SIDE))

# Where each box polarity is delivered (the upper depot of each colour)
DEPOT_FOR = {"N": "left_depot_2_end", "S": "right_depot_2_end"}   # N -> red (left), S -> green (right)
FALLBACK_DEPOT = "right_depot_2_end"   # polarity unknown (no hall sensor yet)

MISSION_TIME_S = 300
HOME_MARGIN_S = 60   # go home once less than this is left


# ---------------------------------------------------------------------------
# Map helpers
# ---------------------------------------------------------------------------
def is_dead_end(node):
    return len(GRAPH[node]) == 1


def direction(a, b):
    """Compass direction of the line from a to its neighbour b."""
    for d, n in GRAPH[a].items():
        if n == b:
            return d
    raise ValueError("%s is not next to %s" % (b, a))


def relative_turn(heading, new_dir):
    """What the robot does to go from facing `heading` to travelling `new_dir`."""
    diff = (DIRS.index(new_dir) - DIRS.index(heading)) % 4
    return ("straight", "right", "reverse", "left")[diff]


def expected_sides(node, heading):
    """Side(s) the outer sensors should see a line on, arriving at node facing heading."""
    i = DIRS.index(heading)
    left = DIRS[(i - 1) % 4] in GRAPH[node]
    right = DIRS[(i + 1) % 4] in GRAPH[node]
    if left and right:
        return "both"
    return "left" if left else ("right" if right else "none")


def path_to_start(node):
    """Nodes from node back to start. The field has no loops, so there is only one way."""
    todo = [[node]]
    while todo:
        path = todo.pop()
        if path[-1] == "start":
            return path
        for nxt in GRAPH[path[-1]].values():
            if nxt and nxt not in path:
                todo.append(path + [nxt])
    raise ValueError("no route from %s to start" % node)


def route(a, b):
    """Nodes from a to b (both included): head towards start until a's and b's routes meet,
    then go out along b's route."""
    up = path_to_start(a)
    down = path_to_start(b)
    for i, node in enumerate(up):
        if node in down:
            j = down.index(node)
            return up[:i + 1] + list(reversed(down[:j]))
    raise ValueError("no route %s -> %s" % (a, b))


def describe(a, b, heading):
    """Human-readable steps from a to b, for checking a route before running it."""
    path = route(a, b)
    lines = []
    for here, nxt in zip(path, path[1:]):
        action = relative_turn(heading, direction(here, nxt))
        lines.append("%-20s facing %s: %s" % (here, heading, action))
        if action != "reverse":
            heading = direction(here, nxt)
    lines.append("%-20s arrive" % path[-1])
    return lines


# ---------------------------------------------------------------------------
# The state machine
# ---------------------------------------------------------------------------
class MissionError(Exception):
    pass


class Mission:
    """Event-driven: the driving code reports what happened, the mission answers what to do.

        expecting_line_end() -> True if the next thing to look for is the end of a branch
        on_junction(seen)    -> "straight" | "left" | "right" | "stop"
                                seen = sides the outer sensors saw: "left" | "right" | "both"
        on_line_end()        -> "collect" | "drop"
        on_job_done(pol)     -> "reverse"   (pol = "N" / "S" / None after collecting)

    States: TO_PICKUP -> TO_DEPOT -> (TO_PICKUP | TO_HOME) -> DONE   (or ERROR)
    """

    def __init__(self, pickups, deliver=True):
        self.pickups = list(pickups)       # branch ends to visit, e.g. ["right_pickup_3_end"]
        self.deliver = deliver             # False: skip the depot, go straight home
        self.polarity = None
        self.delivered = 0
        self.elapsed_s = 0                 # kept up to date by the driving code
        self.heading = "N"                 # robot starts in the box facing the ramp
        self.here = None
        self.state = "TO_PICKUP"
        self.remaining = route("start", self.pickups[0])   # first junction met is the start T

    # ---- events ----------------------------------------------------------
    def on_junction(self, seen):
        node = self._arrive()
        expected = expected_sides(node, self.heading)
        if seen != expected:
            msg = "%s facing %s: expected %s, saw %s" % (node, self.heading, expected, seen)
            if STOP_ON_MISMATCH:
                self.state = "ERROR"
                raise MissionError(msg)
            print("WARNING", msg)

        if not self.remaining:             # last junction of the route: home
            self.state = "DONE"
            return "stop"
        return self._leave_towards(self.remaining[0])

    def on_line_end(self):
        node = self._arrive()
        if not is_dead_end(node):
            self.state = "ERROR"
            raise MissionError("line ended, but %s is not a dead end" % node)
        return "collect" if self.state == "TO_PICKUP" else "drop"

    def on_job_done(self, polarity=None):
        if self.state == "TO_PICKUP":      # just collected
            self.pickups.pop(0)
            self.polarity = polarity
            if self.deliver:
                self._go_to(DEPOT_FOR.get(polarity, FALLBACK_DEPOT), "TO_DEPOT")
            else:
                self._choose_next()
        else:                              # just dropped
            self.delivered += 1
            self.polarity = None
            self._choose_next()
        return self._leave_towards(self.remaining[0])   # out of a dead end: always "reverse"

    def expecting_line_end(self):
        return len(self.remaining) == 1 and is_dead_end(self.remaining[0])

    def finished(self):
        return self.state in ("DONE", "ERROR")

    # ---- internals -------------------------------------------------------
    def _arrive(self):
        if not self.remaining:
            raise MissionError("no more junctions expected")
        self.here = self.remaining.pop(0)
        return self.here

    def _leave_towards(self, nxt):
        d = direction(self.here, nxt)
        action = relative_turn(self.heading, d)
        if action != "reverse":            # reversing keeps the robot facing the same way
            self.heading = d
        return action

    def _go_to(self, target, state):
        self.state = state
        self.remaining = route(self.here, target)[1:]

    def _choose_next(self):
        if self.pickups and MISSION_TIME_S - self.elapsed_s > HOME_MARGIN_S:
            self._go_to(self.pickups[0], "TO_PICKUP")
        else:
            self._go_to("start", "TO_HOME")
