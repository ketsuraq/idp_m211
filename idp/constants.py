"""Shared configuration values for AGV control."""

# Motor speed ceilings.
FORWARD_SPEED = 50
BACKWARD_SPEED = 50
TURN_SPEED = 40
CORRECTION_DIFF = 40
LEFT_SPEED_MULTIPLIER = 1.2   # left wheel is weaker: every left wheel speed is multiplied by this

TURN_TIME = 1.5   # blind part of a junction turn (gets the sensors off the old line)

APPROACH_SPEED = 50
APPROACH_TIME = 1

CORNER_LATENCY = 1.3 # Time to walk forward after a corner before starting to turn to account for difference in position of wheel and sensors
DEPOT_CORNER_LATENCY = 1.4   # same, but at the depot junctions

# Junction detection
CONFIRM_READS = 3            # a junction must show on this many reads in a row
RECHECK_MS = 400             # junction not as expected: keep going this long, re-reading
LEG_TIMEOUT_MS = 20000       # nothing found for this long = lost
TURN_TIMEOUT_MS = 6000       # a junction turn that never finds the new line

# Start box: line follow this long before looking for junctions (gets past the box edge)
START_FOLLOW_MS = 2000

# Turning around at the end of a branch, instead of reversing out (see docs/turnaround_test.md)
TURN_AROUND_BACK_MS = 1000           # pickups only: back up (line following) this long first
TURN_AROUND_SPEED = TURN_SPEED * 2   # spin at double speed
TURN_AROUND_BLIND_TIME = TURN_TIME * 1.5 / 2   # blind part of the spin, halved for the double speed
TURN_AROUND_TIMEOUT_MS = 30000       # keep spinning to search for the line this long before giving up

DEPOT_GAP_MS = 2000 