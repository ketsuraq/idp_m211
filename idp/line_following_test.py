"""First full-route test.

start box -> right pickup 3 -> (collect) -> reverse out -> upper green depot -> (drop)
-> reverse out -> back to the start box.

This file only DRIVES. Every decision (which way to turn, what to do next) comes from
state_machine.Mission. See docs/navigation.md.
"""

from agv import AGV
from constants import *
from main_test import turn_at_junction
from state_machine import Mission
from utime import sleep, sleep_ms, ticks_ms, ticks_diff

PICKUPS = ["right_pickup_3_end"]
CONFIRM_READS = 2            # a junction must show on this many reads in a row
LEG_TIMEOUT_MS = 20000       # nothing found for this long = lost
TURN_TIMEOUT_MS = 6000


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


def check_timeout(agv, start, what):
    if ticks_diff(ticks_ms(), start) > LEG_TIMEOUT_MS:
        agv.stop()
        raise RuntimeError("timed out looking for " + what)


# ---- movements ----------------------------------------------------------------
def wait_until_clear(agv):
    """Keep the current motor command until both outer sensors are off any line."""
    count = 0
    start = ticks_ms()
    while count < CONFIRM_READS:
        r = agv.read_line_sensor()
        count = count + 1 if not (r[0] or r[3]) else 0
        check_timeout(agv, start, "the outer sensors to clear")
        sleep_ms(10)


def pass_junction(agv):
    """Drive straight until both outer sensors are clear of the junction."""
    agv.forward(FORWARD_SPEED)
    wait_until_clear(agv)


def reverse_to_main_line(agv):
    """Reverse out of a branch (line following backwards) until the sensors reach the main line.
    Starts with a plain reverse: at a pickup end the sensors see no line, and at a depot they
    may be on the square's edge, which must not be mistaken for the main line."""
    agv.backward(BACKWARD_SPEED)
    wait_until_clear(agv)
    return drive_to_junction(agv, lambda: agv.reverse_follow_line(BACKWARD_SPEED))


def pivot_to_line(agv, direction):
    """Turn after reversing: no CORNER_LATENCY, short blind turn, then until both middle
    sensors are on the line."""
    if REVERSE_FORWARD_TIME:
        agv.forward(FORWARD_SPEED)
        sleep(REVERSE_FORWARD_TIME)
    agv.turn(direction, TURN_SPEED)
    sleep(REVERSE_TURN_TIME)
    start = ticks_ms()
    while True:
        r = agv.read_line_sensor()
        if r[1] and r[2]:
            break
        if ticks_diff(ticks_ms(), start) > TURN_TIMEOUT_MS:
            agv.stop()
            raise RuntimeError("pivot " + direction + " never found the line")
    agv.stop()


def stop_in_box(agv):
    agv.forward(FORWARD_SPEED)
    sleep(CORNER_LATENCY)   # wheels over the start T
    agv.stop()


# ---- jobs (placeholders until the mechanism and hall sensor are wired in) ---------
def collect(agv):
    print("COLLECT (placeholder)")
    sleep(1)
    return agv.read_hall_sensor()   # None for now -> fallback depot (upper green)


def drop(agv):
    print("DROP (placeholder)")
    sleep(1)


# ---- main loop --------------------------------------------------------------
def run(agv, mission):
    start = ticks_ms()
    just_reversed = False

    # Robot starts in the lower half of the box: drive straight onto the long line (the start T)
    seen = drive_to_junction(agv, lambda: agv.forward(FORWARD_SPEED))
    action = mission.on_junction(seen)

    while True:
        mission.elapsed_s = ticks_diff(ticks_ms(), start) // 1000
        print("[%s] at %s -> %s" % (mission.state, mission.here, action))

        if action == "stop":
            stop_in_box(agv)
            return
        if action == "straight":
            pass_junction(agv)
        elif action in ("left", "right"):
            if just_reversed:
                pivot_to_line(agv, action)
            else:
                turn_at_junction(agv, action)
        just_reversed = False

        if mission.expecting_line_end():
            follow_to_end(agv)
            job = mission.on_line_end()
            polarity = collect(agv) if job == "collect" else drop(agv)
            mission.on_job_done(polarity)           # always "reverse" out of a dead end
            print("[%s] at %s -> reverse" % (mission.state, mission.here))
            seen = reverse_to_main_line(agv)
            just_reversed = True
        else:
            seen = drive_to_junction(agv, lambda: agv.follow_line(FORWARD_SPEED))
        action = mission.on_junction(seen)


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
