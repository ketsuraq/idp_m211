"""Junction turn for when the robot is carrying a box (no forward creep).

The normal turn (line_following_test.turn_at_junction) first drives forward CORNER_LATENCY
so the wheel axle is over the junction, then spins. With a box in front, that creep would
push the box past the junction (into the edge or a wall), so this turn doesn't creep.

Why it can't just spin: when the sensors see the junction, the axle is still a distance d
behind it. A spin on the spot from there ends parallel to the new line but d away from it,
so the sensors never land on it. Instead:
  1. spin a bit less than 90 degrees (BOX_TURN_TIME), so the robot points diagonally at the
     new line
  2. drive forward for BOX_FORWARD_MS without looking at the sensors (gets clear of the
     junction it just turned at), then keep going until a middle sensor meets the new line
  3. keep spinning the same way until both middle sensors are on it
  4. settle: drive forward slowly; whenever a middle sensor loses the line, stop and pivot
     back until both see it again. Each pivot aims the sensors (ahead of the axle) at a
     point further along the line, so the wheels end up closer to it every round. Done
     once both middle sensors stay on the line for BOX_SETTLED_MS. Pivoting on the spot
     fixes the heading without pushing the box forward, unlike follow_line's corrections.

Test on its own: put the robot on a line a little before a junction, facing it, set
TEST_DIRECTION, and run this file. It drives to the junction, does the box turn, then
follows the new line for TEST_FOLLOW_MS.
"""

from agv import AGV
from constants import *
from line_following_test import drive_to_junction
from utime import sleep, sleep_ms, ticks_diff, ticks_ms

# ---- settings (tune on the track) --------------------------------------------------
# Blind spin at the start. Aim for roughly 60-75 degrees. Longer = closer to 90 (less
# forward driving, box goes less past the junction) but if it reaches ~90 the robot ends
# parallel to the line and step 2 never finds it. Shorter = more forward driving and a
# bigger final correction. Starts at the normal turn's blind time; time it on the track.
BOX_TURN_TIME = TURN_TIME/2
BOX_TURN_SPEED = TURN_SPEED              # spin speed for step 1
BOX_INTERCEPT_SPEED = FORWARD_SPEED * 0.6   # step 2 speed. Slower = stops closer to the line
# Step 2 starts with this long driving forward blind (sensors ignored), so the old line or
# the junction can't be mistaken for the new line. Longer = further from the junction
# before looking; too long and it drives over the new line without seeing it (step 2
# then times out). 0 = start looking straight away.
BOX_FORWARD_MS = 300
BOX_INTERCEPT_TIMEOUT_MS = 3000          # step 2 gives up after this (BOX_TURN_TIME too long?)
# Steps 3-4: pivot speed when lining up. Slower = less chance of spinning past "both
# middle sensors on the line" (which makes the next round correct the other way).
BOX_PIVOT_SPEED = TURN_SPEED * 0.6
BOX_SETTLE_SPEED = FORWARD_SPEED * 0.5   # step 4 forward speed between pivots
# Step 4 is done once both middle sensors stay on the line this long while driving.
# Longer = straighter before handing over, but uses more of the line (mind the next junction).
BOX_SETTLED_MS = 400
BOX_ALIGN_TIMEOUT_MS = 3000              # any single pivot gives up after this
BOX_SETTLE_TIMEOUT_MS = 5000             # step 4 as a whole gives up after this

# ---- test settings ----------------------------------------------------------------
TEST_DIRECTION = "left"                  # "left" or "right"
TEST_FOLLOW_MS = 2000                    # line follow this long after the turn


def turn_with_box(agv, direction):
    """Turn onto the line to the `direction` side ("left"/"right") at a junction the
    sensors have just reached, without driving forward first. See the module docstring."""
    agv.stop()

    # 1. Partial spin: point diagonally towards the new line
    print("box turn: spinning", direction)
    agv.turn(direction, BOX_TURN_SPEED)
    sleep(BOX_TURN_TIME)
    agv.stop()

    # 2. Drive forward blind for BOX_FORWARD_MS, then until a middle sensor meets the new line
    print("box turn: driving forward")
    agv.forward(BOX_INTERCEPT_SPEED)
    #sleep_ms(BOX_FORWARD_MS)
    print("box turn: looking for the new line")
    start = ticks_ms()
    while True:
        r = agv.read_line_sensor()
        if r[1] or r[2]:
            break
        if ticks_diff(ticks_ms(), start) > BOX_INTERCEPT_TIMEOUT_MS:
            agv.stop()
            raise RuntimeError("box turn: never reached the new line")
        sleep_ms(5)
    agv.stop()

    while True:
        agv.line_follow(correction=CORRECTION_DIFF+20)




def main():
    agv = AGV()
    try:
        print("Driving to the junction...")
        drive_to_junction(agv, lambda: agv.follow_line(FORWARD_SPEED))
        turn_with_box(agv, TEST_DIRECTION)

        print("Following the new line...")
        agv.forward(FORWARD_SPEED)
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < TEST_FOLLOW_MS:
            agv.follow_line(FORWARD_SPEED)
            sleep_ms(10)
    except KeyboardInterrupt:
        print("Ctrl-C")
    except Exception as e:
        print("Error:", e)
    finally:
        agv.stop()


if __name__ == "__main__":
    main()
