"""First competition run.

BEHAVIOUR
=========
The robot starts in the start box facing the ramp. Amber flashes for the whole run.

The cycle (repeated until it's time to go home):
    1. lower left  branches 1-6   -> deliver the box found there
    2. lower right branches 1-6   -> deliver
    3. upper left  branches 1-7   -> deliver   (up the ramp: waits in the box for green)
    4. upper right branches 1-7   -> deliver   (same)
    then back to 1 with the memory cleared (new boxes may have been placed).
Branch 1 is the one nearest the corner (lower) or the ramp (upper); upper 7 is the top T.

Delivering: a north-pole box goes to the left (red) depot, south-pole to the right (green)
one (state_machine.DEPOT_FOR). While a box is carried the red (N) or green (S) LED is on.

Scanning one side
-----------------
Way out (1 -> 6, or 1 -> 7 upstairs). The robot drives down every branch to the end:
  - box seen and its pole read      -> take it to the correct depot. Side done.
  - box seen but no pole read       -> leave it, remember the branch number.
  - no box                          -> next branch.
Way back (5 -> 1), only if nothing was taken on the way out. Every branch again:
  - box with a pole                 -> take it to the correct depot. Side done.
  - box, still no pole, and a box remembered from the way out lies further down
                                    -> leave it, carry on down (that one gets a go first).
  - box, still no pole, and no remembered box further down (it's the last one)
                                    -> take it anyway, read the pole again once gripped:
                                       pole -> correct depot, still none -> the depot on
                                       the same side as the branch ("closest depot").
  So with two unread boxes (e.g. at 4 and 2), 4 is rechecked and left, 2 is taken.
  If the last remembered box isn't seen again, it keeps going down to branch 1.
Reached branch 1 with nothing taken:
  - a box was seen (no pole) on the way back, or at the top branch (which the way back
    starts below) -> go back up to the lowest such branch and take it, closest depot.
  - nothing seen at all -> no depot, go on to the next side.

Going home
----------
From HOME_AT_S (4:30) the robot heads for the start box and stops inside it:
  - not carrying a box: straight home.
  - carrying a box: compare the points. Going home is worth 15 if 2+ boxes have been
    delivered, else 0. The delivery is worth 15 (pole known) or 5 (no pole). Home only if
    it's worth more; otherwise (a tie too) deliver first, then go home.
The time is checked at every junction and after every branch or drop. If going home
would mean reversing on the main line, it waits until the end of the current branch.

Stop button: stops the robot at any time; all LEDs go off.

Needs on the Pico: competition_1.py, main.py, state_machine.py, constants.py, agv.py,
hardware.py, VL53L0X.py. It doesn't use grab_test.py or line_following_test.py: the
driving helpers below are copies of theirs.
"""

from agv import AGV
from constants import *
from state_machine import (
    DEPOT_FOR, OPPOSITE, STOP_ON_MISMATCH, UPPER_CROSSINGS, Mission, MissionError,
    direction, expected_sides, is_dead_end, relative_turn, route,
)
from utime import sleep_ms, ticks_diff, ticks_ms

# ---- settings -----------------------------------------------------------------------
# Head home from this many seconds after the start (4:30). The run is 5:00, so this
# leaves 30 s to get back. Raise it to fit more in, lower it if the robot gets home late.
HOME_AT_S = 270

# A box counts once the distance sensor reads below BOX_DETECT_MM this many times in a
# row, so a single bad reading isn't a box. Higher = fewer false boxes, but a box seen
# only briefly might be missed.
BOX_CONFIRM_READS = 2

# Points used for the 4:30 decision (from the README's scoring)
HOME_POINTS = 15             # back in the start box after delivering 2+ boxes
HOME_MIN_DELIVERED = 2       # deliveries needed before going home scores anything
CORRECT_DEPOT_POINTS = 15    # pole known, right depot
WRONG_DEPOT_POINTS = 5       # no pole: closest depot, may well be the wrong one

# The four sides, in the order they're scanned: (map name prefix, number of branches)
SIDES = [
    ("left", 6),
    ("right", 6),
    ("upper_left", UPPER_CROSSINGS + 1),
    ("upper_right", UPPER_CROSSINGS + 1),
]


def branch_node(side, n):
    """Map node at the end of branch n on a side: "left_pickup_3_end", "upper_left_3_end"."""
    if side.startswith("upper_"):
        return "%s_%d_end" % (side, n)
    return "%s_pickup_%d_end" % (side, n)


def closest_depot(side):
    """Depot for a box whose pole can't be read: the one on the same side as the branch."""
    return ("left" if "left" in side else "right") + "_depot_2_end"


# ---- mission (decisions only, no hardware) ---------------------------------------------
class CompetitionMission(Mission):
    """Decides which branch to check next, where to deliver and when to go home.
    Routing, turns and junction checks come from state_machine.Mission.

    The driving code reports events and gets back what to do:
        on_junction(seen)                    -> "straight" | "left" | "right" | "stop"
        take_without_pole()                  -> True if a box with no pole should be taken
        on_branch_done(found, pole, taken)   -> "reverse" (turn around, go to the next place)
        on_depot_done()                      -> "reverse" (reverse out, go to the next place)
    """

    def __init__(self, clock=None):
        start = ticks_ms()
        # seconds since the start; replaceable for a dry run
        self.clock = clock or (lambda: ticks_diff(ticks_ms(), start) / 1000)
        self.delivered = 0
        self.polarity = None          # pole of the box being carried ("N", "S" or None)
        self.carrying = False
        self.home_decided = False     # the 4:30 decision has been made
        self.heading = "N"            # starts in the box facing the ramp
        self.here = "start"
        self.side_index = -1
        self._start_next_side()
        self.here = None              # the first junction met is the start T itself
        self.remaining = route("start", branch_node(self.side, 1))

    # ---- which branch next ---------------------------------------------------------
    def _start_next_side(self):
        """Move on to the next side in SIDES (wrapping round) and head for its branch 1."""
        self.side_index = (self.side_index + 1) % len(SIDES)
        self.side, self.branches = SIDES[self.side_index]
        self.scan = "out"             # "out" (1 -> N), "back" (N-1 -> 1), "fetch" (go and get one)
        self.n = 1                    # branch being checked
        self.seen_out = []            # branches with an unread box, on the way out
        self.seen_back = []           # same, on the way back
        print("--- side:", self.side)
        self._go_to(branch_node(self.side, 1), "TO_PICKUP")

    def _next_branch(self):
        """Branch number to check next on this side, or None when the side is done."""
        if self.scan == "out":
            if self.n < self.branches:
                return self.n + 1
            self.scan = "back"
            return self.branches - 1
        if self.scan == "back":
            if self.n > 1:
                return self.n - 1
            # Down to 1 with nothing taken. The top branch was never rechecked (the way
            # back starts below it), so an unread box there still counts.
            left_behind = self.seen_back + [b for b in self.seen_out if b == self.branches]
            if left_behind:
                self.scan = "fetch"
                return min(left_behind)   # nearest to branch 1
        return None                       # "fetch" done, or nothing seen at all

    def take_without_pole(self):
        """A box is here but its pole couldn't be read: take it anyway?"""
        if self.scan == "fetch":
            return True
        if self.scan == "back":
            # only if no box remembered from the way out is still further down
            return not any(b < self.n for b in self.seen_out)
        return False

    # ---- events --------------------------------------------------------------------
    def on_junction(self, seen):
        node = self._arrive()
        expected = expected_sides(node, self.heading)
        if seen != expected:
            msg = "%s facing %s: expected %s, saw %s" % (node, self.heading, expected, seen)
            if STOP_ON_MISMATCH:
                self.state = "ERROR"
                raise MissionError(msg)
            print("WARNING", msg)
        self._check_time()
        if not self.remaining:            # at the start T on the way home
            self.state = "DONE"
            return "stop"
        return self._leave_towards(self.remaining[0])

    def on_branch_done(self, found, pole, taken):
        """At the end of branch self.n: found = a box was seen, pole = "N"/"S"/None,
        taken = the box is now in the gripper."""
        if taken:
            self.carrying = True
            self.polarity = pole
            depot = DEPOT_FOR[pole] if pole else closest_depot(self.side)
            print("taking box (pole %s) to %s" % (pole, depot))
            self._go_to(depot, "TO_DEPOT")
        else:
            if found:
                (self.seen_out if self.scan == "out" else self.seen_back).append(self.n)
            nxt = self._next_branch()
            if nxt:
                self.n = nxt
                self._go_to(branch_node(self.side, nxt), "TO_PICKUP")
            else:
                print("nothing to take on", self.side)
                self._start_next_side()
        self._check_time()
        return self._leave_towards(self.remaining[0])

    def on_depot_done(self):
        self.delivered += 1
        self.carrying = False
        self.polarity = None
        if self.home_decided or self.clock() >= HOME_AT_S:
            self._go_to("start", "TO_HOME")
        else:
            self._start_next_side()
        return self._leave_towards(self.remaining[0])

    # ---- going home ----------------------------------------------------------------
    def _check_time(self):
        """From HOME_AT_S, send the robot home (see 'Going home' at the top)."""
        if self.state in ("TO_HOME", "DONE") or self.home_decided:
            return
        if self.clock() < HOME_AT_S:
            return
        if self.carrying:
            home = HOME_POINTS if self.delivered >= HOME_MIN_DELIVERED else 0
            deliver = CORRECT_DEPOT_POINTS if self.polarity else WRONG_DEPOT_POINTS
            if home <= deliver:
                print("time's up: delivering first (%d pts) then home" % deliver)
                self.home_decided = True
                return
        if self._go_home():
            print("time's up: going home")
            self.home_decided = True

    def _go_home(self):
        """Route home from here, unless that means reversing on the main line (the robot
        only turns round at the end of a branch): then wait for the next event."""
        path = route(self.here, "start")
        if len(path) > 1 and not is_dead_end(self.here):
            if relative_turn(self.heading, direction(self.here, path[1])) == "reverse":
                return False
        self._go_to("start", "TO_HOME")
        return True


# ---- sensing ---------------------------------------------------------------------------
def sides(readings):
    """Which outer sensors see a line: "left", "right", "both" or None."""
    left, right = readings[0], readings[3]
    if left and right:
        return "both"
    if left:
        return "left"
    if right:
        return "right"
    return None


def check_timeout(agv, start, what):
    """Give up (stop, raise) if a leg has taken longer than LEG_TIMEOUT_MS: the robot is lost."""
    if ticks_diff(ticks_ms(), start) > LEG_TIMEOUT_MS:
        agv.stop()
        raise RuntimeError("timed out looking for " + what)


def drive_to_junction(agv, drive):
    """Call drive() (e.g. follow the line) until an outer sensor sees a line on
    CONFIRM_READS reads in a row. Returns the sides seen: "left", "right" or "both"."""
    count = 0
    seen_left = seen_right = False
    start = ticks_ms()
    while True:
        agv.check_stop_button()
        s = sides(agv.read_line_sensor())
        if s:
            count += 1
            seen_left = seen_left or s in ("left", "both")
            seen_right = seen_right or s in ("right", "both")
            if count >= CONFIRM_READS:
                return "both" if seen_left and seen_right else ("left" if seen_left else "right")
        else:
            count = 0
            seen_left = seen_right = False
            drive()
        check_timeout(agv, start, "a junction")
        sleep_ms(10)


def recheck(agv, mission, seen):
    """If the junction doesn't look as expected, keep the current motor action for up to
    RECHECK_MS, re-reading the sensors until it does. Sides add up over time: "right" seen
    earlier plus "left" now counts as "both" (robot crossing the line at an angle)."""
    expected = expected_sides(mission.remaining[0], mission.heading)
    start = ticks_ms()
    while seen != expected and ticks_diff(ticks_ms(), start) < RECHECK_MS:
        sleep_ms(10)
        agv.check_stop_button()
        s = sides(agv.read_line_sensor())
        if s and s != seen:
            seen = "both"
    return seen


def drive_to_depot(agv):
    """Coming down the lower main line with a box: find the depot junction as the first
    junction after a gap of at least DEPOT_GAP_MS (the gap between pickup 1 and the depot
    is the longest on that line). Returns the sides seen there."""
    start = ticks_ms()
    clear_start = None
    count = 0
    on_junction = False
    agv.forward(FORWARD_SPEED)
    while True:
        agv.check_stop_button()
        detected = sides(agv.read_line_sensor())
        if detected:
            count += 1
            if count >= CONFIRM_READS and not on_junction:
                on_junction = True
                if clear_start is not None:
                    gap = ticks_diff(ticks_ms(), clear_start)
                    print("junction gap:", gap, "ms")
                    if gap >= DEPOT_GAP_MS:
                        return detected
        else:
            if on_junction or clear_start is None:
                clear_start = ticks_ms()
            on_junction = False
            count = 0
        agv.follow_line(FORWARD_SPEED)
        check_timeout(agv, start, "the depot junction")
        sleep_ms(10)


def follow_to_end(agv):
    """Follow the branch until the line ends: no line, or a bar across all four sensors
    (e.g. the edge of a depot square), on CONFIRM_READS reads in a row."""
    count = 0
    start = ticks_ms()
    while count < CONFIRM_READS:
        agv.check_stop_button()
        r = agv.read_line_sensor()
        if not any(r) or all(r):
            count += 1
        else:
            count = 0
            agv.follow_line(FORWARD_SPEED)
        check_timeout(agv, start, "the end of the line")
        sleep_ms(10)
    agv.stop()


def follow_branch(agv):
    """Follow a pickup branch to the end of the line, watching for a box on the way.
    It keeps going to the end either way, so every branch finishes in the same place.
    Returns (found, mm): mm is the reading that confirmed the box, or None."""
    end_count = 0
    box_count = 0
    found_mm = None
    start = ticks_ms()
    while True:
        agv.check_stop_button()
        if found_mm is None:   # stop reading once found
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


def read_pole(agv):
    """The box's pole as the map writes it: "N", "S" or None (couldn't read one)."""
    return {"north": "N", "south": "S"}.get(agv.detect_pole())


# ---- movements -------------------------------------------------------------------------
def pass_junction(agv):
    """Drive straight until both outer sensors are clear of the junction."""
    agv.forward(FORWARD_SPEED)
    count = 0
    start = ticks_ms()
    while count < CONFIRM_READS:
        agv.check_stop_button()
        r = agv.read_line_sensor()
        count = count + 1 if not (r[0] or r[3]) else 0
        check_timeout(agv, start, "the outer sensors to clear")
        sleep_ms(10)


def turn_at_junction(agv, direction, latency=CORNER_LATENCY):
    """Drive forward `latency` seconds so the wheels reach the junction, then pivot onto
    the new line. latency=0 (axle already over the junction, e.g. out of a depot) skips
    the forward part."""
    # 1. Walk forward (line following) to put the wheel axle over the junction
    if latency > 0:
        agv.forward(FORWARD_SPEED)
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < latency * 1000:
        agv.check_stop_button()
        agv.follow_line(FORWARD_SPEED)
        sleep_ms(10)
    # 2. Blind turn to get the sensors off the old line
    agv.turn(direction, TURN_SPEED)
    agv.interruptible_sleep(int(TURN_TIME * 1000))
    # 3. Keep turning until both middle sensors confirm the new line
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
    # 4. Keep turning a moment after confirmation, then stop
    agv.interruptible_sleep(TURN_ALIGN_SETTLE_MS)
    agv.stop()


def back_off_line(agv):
    """If an outer sensor is already on a line after turning out of a branch (the turn
    carried the sensors onto the next junction), reverse until both outer sensors are
    clear, then set off forwards, so that junction is approached from behind."""
    r = agv.read_line_sensor()
    if not (r[0] or r[3]):
        return
    print("outer sensor already on a line: backing off it")
    agv.backward(BACKWARD_SPEED)
    count = 0
    start = ticks_ms()
    while count < CONFIRM_READS:
        agv.check_stop_button()
        r = agv.read_line_sensor()
        count = count + 1 if not (r[0] or r[3]) else 0
        agv.reverse_follow_line(BACKWARD_SPEED)
        check_timeout(agv, start, "the outer sensors to clear (backing off)")
        sleep_ms(10)
    agv.forward(FORWARD_SPEED)


def back_up(agv, ms):
    """Reverse (line following) for ms milliseconds, then stop."""
    agv.backward(BACKWARD_SPEED)   # no line under the sensors at a pickup end: keep reversing
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < ms:
        agv.check_stop_button()
        agv.reverse_follow_line(BACKWARD_SPEED)
        sleep_ms(10)
    agv.stop()


def turn_around(agv, backing_up=True):
    """At the end of a branch: back up for TURN_AROUND_BACK_MS (unless already done), then
    spin left until the middle sensors leave the line and find it again, i.e. facing back
    the way it came."""
    if backing_up:
        print("turn around: backing up")
        back_up(agv, TURN_AROUND_BACK_MS)
    print("turn around: spinning")
    agv.turn("left", TURN_AROUND_SPEED)
    agv.interruptible_sleep(int(TURN_AROUND_BLIND_TIME * 1000))
    start = ticks_ms()
    try:
        # No sleep in these two loops: spinning fast, the middle sensors are only both on
        # the line for a moment, so poll as fast as possible
        while agv.centred_on_line():
            agv.check_stop_button()
            if ticks_diff(ticks_ms(), start) > TURN_AROUND_TIMEOUT_MS:
                raise RuntimeError("Turn around timeout (never left the line)")
        while not agv.centred_on_line():
            agv.check_stop_button()
            if ticks_diff(ticks_ms(), start) > TURN_AROUND_TIMEOUT_MS:
                raise RuntimeError("Turn around timeout (never found the line)")
    finally:
        agv.stop()


def reverse_out_of_depot(agv):
    """After a drop: back up DEPOT_BACKWARDS_TIME seconds, leaving the wheel axle over the
    main line. It doesn't turn: the mission gives the turn at the depot junction."""
    back_up(agv, DEPOT_BACKWARDS_TIME * 1000)


def stop_in_box(agv):
    """At the start T: drive the wheels onto the T and stop (inside the start box)."""
    agv.forward(FORWARD_SPEED)
    agv.interruptible_sleep(int(CORNER_LATENCY * 1000))
    agv.stop()


def wait_for_green(agv):
    """Stop and block until the traffic light's green IR signal: the receiver's output held
    low for IR_LOW_HOLD_MS (green is lows of about 80 ms; shorter glitches are ignored)."""
    agv.stop()
    low_since = None
    while True:
        agv.check_stop_button()
        if agv.read_ir_sensor():                    # True = low = signal
            if low_since is None:
                low_since = ticks_ms()
            elif ticks_diff(ticks_ms(), low_since) >= IR_LOW_HOLD_MS:
                return
        else:
            low_since = None
        sleep_ms(IR_SAMPLE_MS)


def go_up_ramp(agv, action):
    """At the start T, heading for the ramp: end up over the T facing N (drive on if
    already facing N, else turn), wait for green, then drive clear of the T. The box edge
    is then found as a normal junction."""
    if action == "straight":
        stop_in_box(agv)
    else:
        turn_at_junction(agv, action, CORNER_LATENCY)
    print("waiting for green")
    wait_for_green(agv)
    print("green: going up the ramp")
    pass_junction(agv)


def leave_start_box(agv, mission):
    """After turning E/W out of the start T: cross the box edge without treating it as a
    junction (it's too close to the turn). Junctions count again once START_EXIT_IGNORE_MS
    has passed and the robot is centred with both outer sensors clear. The mission is then
    told it passed the box edge."""
    print("leaving start box - ignoring the box edge")
    agv.forward(FORWARD_SPEED)
    start = ticks_ms()
    clear_count = 0
    while True:
        agv.check_stop_button()
        readings = agv.read_line_sensor()
        elapsed = ticks_diff(ticks_ms(), start)
        agv.follow_line(FORWARD_SPEED)
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
    action = mission.on_junction("both")
    if action != "straight":
        agv.stop()
        raise RuntimeError("Expected straight at box edge, got " + action)
    agv.forward(FORWARD_SPEED)


# ---- jobs ------------------------------------------------------------------------------
def examine_box(agv, mission):
    """At the end of a branch with a box: read its pole. Pole -> grab it. No pole -> grab
    it only if the mission says so (last box on the way back), then read again now it's
    gripped. Returns (pole, taken)."""
    pole = read_pole(agv)
    if pole:
        print("pole:", pole)
    elif mission.take_without_pole():
        print("no pole, taking it anyway")
        agv.close_gripper()
        pole = read_pole(agv)
        print("pole once gripped:", pole)
        if pole:
            agv.show_polarity(pole)
        return pole, True
    else:
        print("no pole, leaving it")
        return None, False
    agv.show_polarity(pole)           # steady red (N) / green (S) until the drop
    agv.close_gripper()
    return pole, True


def back_out(agv):
    """After grabbing: back up the same distance as turn_around would, so the spin
    afterwards starts from the same place as at an empty branch. (The gripper isn't
    lowered here, same as grab_test.)"""
    print("grab: backing out")
    back_up(agv, TURN_AROUND_BACK_MS)


def drop(agv):
    """At the end of the depot line: let go of the box (red/green off), raise the arm."""
    print("DROP")
    agv.open_gripper()
    agv.show_polarity(None)
    agv.lift()


# ---- main loop -------------------------------------------------------------------------
def run(agv, mission):
    """Run the mission with amber flashing; every LED goes off at the end, however it
    ends (home, stop button or an error)."""
    agv.start_mission_lights()
    try:
        _drive(agv, mission)
    finally:
        agv.stop_mission_lights()


def _drive(agv, mission):
    """Drive from the start box, asking the mission what to do at every junction and at
    the end of every branch."""
    agv.lift()           # no box yet: arm raised
    agv.open_gripper()   # and gripper open, ready to close on the first box
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < START_FOLLOW_MS:   # get going towards the start T
        agv.check_stop_button()
        agv.follow_line(FORWARD_SPEED)
        sleep_ms(5)

    seen = drive_to_junction(agv, lambda: agv.follow_line(FORWARD_SPEED))
    action = mission.on_junction(recheck(agv, mission, seen))
    out_of_pickup = False        # True right after turning around at a pickup branch
    looking_for_depot = False

    while True:
        agv.check_stop_button()
        print("[%s %ds] at %s -> %s" % (mission.state, mission.clock(), mission.here, action))
        out_of_branch = False    # this pass turned from a branch onto the main line

        if action == "stop":     # at the start T on the way home
            stop_in_box(agv)
            print("home. delivered:", mission.delivered)
            return
        if mission.must_wait_for_green():
            go_up_ramp(agv, action)
        elif action == "straight":
            pass_junction(agv)
        elif action in ("left", "right"):
            latency = DEPOT_CORNER_LATENCY if "depot" in mission.here else CORNER_LATENCY
            turn_at_junction(agv, action, latency)
            if mission.here == "start":
                leave_start_box(agv, mission)
            out_of_branch = out_of_pickup
            if out_of_pickup and mission.state == "TO_PICKUP":
                # On to another branch: back up a little so its junction is approached
                # from behind
                back_up(agv, BETWEEN_PICKUP_BACK_MS)
            # The depot is found by the long gap before it. Not from the upper deck: the
            # ramp is a long gap too, so there the map's junction count is used.
            if out_of_pickup and mission.state == "TO_DEPOT" and not mission.here.startswith("upper_"):
                looking_for_depot = True
        out_of_pickup = False

        if mission.expecting_line_end():
            if mission.state == "TO_PICKUP":
                found, mm = follow_branch(agv)
                mission.on_line_end()
                print("[%s] %s: %s" % (mission.state, mission.here,
                                       "box at %d mm" % mm if found else "empty"))
                pole, taken = examine_box(agv, mission) if found else (None, False)
                action = mission.on_branch_done(found, pole, taken)
                print("[%s] at %s -> turn around" % (mission.state, mission.here))
                if taken:   # holding the box: back out, then spin
                    back_out(agv)
                    turn_around(agv, backing_up=False)
                else:
                    turn_around(agv, backing_up=True)
                mission.heading = OPPOSITE[mission.heading]   # facing back the way it came
                out_of_pickup = True
            else:   # depot: drop, reverse out, turn onto the main line
                follow_to_end(agv)
                mission.on_line_end()
                drop(agv)
                mission.on_depot_done()
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

        if out_of_branch:
            back_off_line(agv)

        if looking_for_depot:
            seen = drive_to_depot(agv)
            # Skip the route ahead to the depot junction drive_to_depot found
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
    mission = CompetitionMission()
    try:
        run(agv, mission)
    except KeyboardInterrupt:
        print("Ctrl-C")
    except Exception as e:
        print("Error:", e)
    finally:
        agv.stop()
        print("Finished in state", mission.state, "| delivered:", mission.delivered)


if __name__ == "__main__":
    main()
