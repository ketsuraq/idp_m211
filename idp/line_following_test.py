"""First full-route test.

start box -> right pickup 3 -> (collect) -> turn around -> upper green depot -> (drop)
-> turn around -> back to the start box.

This file only DRIVES. Every decision (which way to turn, what to do next) comes from
state_machine.Mission. See docs/navigation.md and docs/turnaround_test.md.
"""

from agv import AGV
from constants import *
from state_machine import Mission, OPPOSITE, direction, expected_sides
from utime import sleep, sleep_ms, ticks_ms, ticks_diff

PICKUPS = [
    "left_pickup_1_end",
    "left_pickup_2_end",
    "left_pickup_3_end",
    "left_pickup_4_end",
    "left_pickup_5_end",
    "left_pickup_6_end"
]



# ---- sensing --------------------------------------------------------------
def sides(readings):
    left, right = readings[0], readings[3]
    if left and right:
        return "both"
    if left:
        return "left"
    if right:
        return "right"
    return None
    

def turn_at_junction(agv, direction, latency=CORNER_LATENCY):
    """Drive forward so the wheels reach the junction, then pivot onto the new line."""
    # 1. Walk forward (line following) to put the wheel axle over the junction.
    # Where there's no line ahead, follow_line keeps the last command, so start going straight.
    agv.forward(FORWARD_SPEED)
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < latency * 1000:
        agv.follow_line(FORWARD_SPEED)
        sleep_ms(10)

    # 2. Blind turn to get the sensors off the old line
    agv.turn(direction, TURN_SPEED)
    sleep(TURN_TIME)

    # 3. Keep turning until either middle sensor finds the new line
    start = ticks_ms()
    while True:
        readings = agv.read_line_sensor()
        if readings[1] and readings[2]:
            break
        if ticks_diff(ticks_ms(), start) > TURN_TIMEOUT_MS:
            raise RuntimeError("Turn timed out without finding the line")
    agv.stop()

def drive_to_junction(agv, drive):
    """Call drive() (e.g. follow the line) until an outer sensor sees a line on
    CONFIRM_READS reads in a row. Returns the sides seen: "left", "right" or "both"."""
    count = 0
    seen_left = seen_right = False
    start = ticks_ms()
    while True:
        r = agv.read_line_sensor()
        s = sides(r)
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


def follow_to_end(agv):
    """Follow the branch until the line ends: no line, or a bar across all four sensors
    (e.g. the edge of a depot square), on CONFIRM_READS reads in a row."""
    count = 0
    start = ticks_ms()
    while count < CONFIRM_READS:
        r = agv.read_line_sensor()
        if not any(r) or all(r):
            count += 1
        else:
            count = 0
            agv.follow_line(FORWARD_SPEED)
        check_timeout(agv, start, "the end of the line")
        sleep_ms(10)
    agv.stop()

def drive_to_depot(agv):
    """Find depot junction using the long gap between branches."""

    start = ticks_ms()
    clear_start = None
    count = 0
    on_junction = False

    while True:
        readings = agv.read_line_sensor()
        detected = sides(readings)

        if detected:
            count += 1

            if count >= CONFIRM_READS and not on_junction:
                on_junction = True

                if clear_start is not None:
                    gap = ticks_diff(ticks_ms(), clear_start)
                    print("Junction gap:", gap, "ms")

                    if gap >= DEPOT_GAP_MS:
                        return detected

        else:
            if on_junction or clear_start is None:
                clear_start = ticks_ms()

            on_junction = False
            count = 0

        agv.follow_line(FORWARD_SPEED)
        check_timeout(agv, start, "depot junction")
        sleep_ms(10)



def recheck(agv, mission, seen):
    """If the junction doesn't look as expected, keep the current motor action for up to
    RECHECK_MS, re-reading the sensors until it does. Sides add up over time: "right" seen
    earlier plus "left" now counts as "both" (robot crossing the line at an angle)."""
    expected = expected_sides(mission.remaining[0], mission.heading)
    start = ticks_ms()
    while seen != expected and ticks_diff(ticks_ms(), start) < RECHECK_MS:
        sleep_ms(10)
        s = sides(agv.read_line_sensor())
        if s and s != seen:
            seen = "both"
    return seen


def check_timeout(agv, start, what):
    if ticks_diff(ticks_ms(), start) > LEG_TIMEOUT_MS:
        agv.stop()
        raise RuntimeError("timed out looking for " + what)


# ---- movements ----------------------------------------------------------------
def pass_junction(agv):
    """Drive straight until both outer sensors are clear of the junction."""
    agv.forward(FORWARD_SPEED)
    count = 0
    start = ticks_ms()
    while count < CONFIRM_READS:
        r = agv.read_line_sensor()
        count = count + 1 if not (r[0] or r[3]) else 0
        check_timeout(agv, start, "the outer sensors to clear")
        sleep_ms(10)


def turn_around(agv, back_up=True):
    """At the end of a branch: back up (line following) for TURN_AROUND_BACK_MS, then spin
    left at TURN_AROUND_SPEED. Blind for TURN_AROUND_BLIND_TIME, then keep spinning until the
    middle sensors leave the line and find it again, i.e. facing back the way it came."""
    if back_up:   # not at depots: their lines are short
        print("turn around: backing up")
        agv.backward(BACKWARD_SPEED)   # no line under the sensors at a pickup end: keep reversing
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < TURN_AROUND_BACK_MS:
            agv.reverse_follow_line(BACKWARD_SPEED)
            sleep_ms(10)
    print("turn around: spinning (blind)")
    agv.turn("left", TURN_AROUND_SPEED)
    sleep(TURN_AROUND_BLIND_TIME)
    start = ticks_ms()
    try:
        # No sleep in these two loops: spinning fast, the middle sensors are only both on
        # the line for a moment, so poll as fast as possible
        print("turn around: waiting for middle sensors to leave the line")
        while agv.centred_on_line():
            if ticks_diff(ticks_ms(), start) > TURN_AROUND_TIMEOUT_MS:
                raise RuntimeError("Turn around timeout (never left the line)")
        print("turn around: off the line, looking for it again")
        while not agv.centred_on_line():
            if ticks_diff(ticks_ms(), start) > TURN_AROUND_TIMEOUT_MS:
                raise RuntimeError("Turn around timeout (never found the line)")
        print("turn around: done, back on the line")
    finally:
        agv.stop()


def stop_in_box(agv):
    agv.forward(FORWARD_SPEED)
    sleep(CORNER_LATENCY)   # wheels over the start T
    agv.stop()


# ---- jobs (placeholders until the mechanism and hall sensor are wired in) ---------
def collect(agv, station):
    """Check whether a box exists at this station."""

    print("Checking:", station)

    # Simulated distance sensor
    box_detected = station == "left_pickup_6_end"

    if not box_detected:
        print("No box found")
        return False, None

    print("Box found! Collecting...")
    sleep(1)

    return True, "N"
    # return agv.read_hall_sensor()   # None for now -> fallback depot (upper green)


def drop(agv):
    print("DROP (placeholder)")
    sleep(1)


# ---- main loop --------------------------------------------------------------
def run(agv, mission, from_box=True):
    """Drive the mission. from_box: the robot starts in the start box, so line follow for
    START_FOLLOW_MS first. Otherwise (mid-route tests) it starts on a line, right away."""
    start = ticks_ms()

    out_of_pickup = False
    looking_for_depot = False

    if from_box:
        while ticks_diff(ticks_ms(), start) < START_FOLLOW_MS:
            agv.follow_line(FORWARD_SPEED)
            sleep_ms(10)

    # Now start looking for junctions
    seen = drive_to_junction(agv, lambda: agv.follow_line(FORWARD_SPEED))
    action = mission.on_junction(recheck(agv, mission, seen))
    out_of_pickup = False

    while True:
        mission.elapsed_s = ticks_diff(ticks_ms(), start) // 1000
        print("[%s] at %s -> %s" % (mission.state, mission.here, action))

        if action == "stop":
            stop_in_box(agv)
            return
        if action == "straight":
            pass_junction(agv)
        
        elif action in ("left", "right"):
            latency = DEPOT_CORNER_LATENCY if "depot" in mission.here else CORNER_LATENCY
            turn_at_junction(agv, action, latency)


            if out_of_pickup and mission.state == "TO_PICKUP":
                agv.backward(BACKWARD_SPEED)
                reverse_start = ticks_ms()

                while ticks_diff(ticks_ms(), reverse_start) < BETWEEN_PICKUP_BACK_MS:
                    agv.reverse_follow_line(BACKWARD_SPEED)
                    sleep_ms(10)

                agv.stop()


            if out_of_pickup and mission.state == "TO_DEPOT":
                looking_for_depot = True

        out_of_pickup = False


        if mission.expecting_line_end():
            follow_to_end(agv)
            job = mission.on_line_end()

            if job == "collect":
                box_found, polarity = collect(agv, mission.here)
                mission.on_job_done(polarity, box_found=box_found)

            else:
                drop(agv)
                mission.on_job_done()

            print("[%s] at %s -> turn around" % (mission.state, mission.here))
            turn_around(agv, back_up=(job == "collect"))   # no backing up at depots
            out_of_pickup = job == "collect"
            mission.heading = OPPOSITE[mission.heading]   # now facing back the way it came
            print("[%s] turned around, now facing %s" % (mission.state, mission.heading))
            if job == "drop":
                # Depot lines are short: the spin lands it on the main line, already facing the
                # next junction, so the depot junction and its turn are done
                mission.here = mission.remaining.pop(0)
                mission.heading = direction(mission.here, mission.remaining[0])
                print("[%s] out of the depot onto the main line at %s, facing %s" % (mission.state, mission.here, mission.heading))
    
        if looking_for_depot:
            seen = drive_to_depot(agv)

            # Synchronise the route with the depot junction
            for i, node in enumerate(mission.remaining):
                if "depot" in node and not node.endswith("_end"):
                    mission.remaining = mission.remaining[i:]
                    break
            else:
                raise RuntimeError("No depot junction in route")

            looking_for_depot = False

        else:
            seen = drive_to_junction(
                agv,
                lambda: agv.follow_line(FORWARD_SPEED)
            )

        action = mission.on_junction(recheck(agv, mission, seen))



def main():
    agv = AGV()
    mission = Mission(PICKUPS)
    try:
        run(agv, mission)
    except KeyboardInterrupt:
        print("Ctrl-C")
    except Exception as e:
        print("Error:", e)
    finally:
        agv.stop()
        print("Finished in state", mission.state)


if __name__ == "__main__":
    main()
