"""Shared configuration values for AGV control."""

# Motor speed ceilings.
FORWARD_SPEED = 80
BACKWARD_SPEED = FORWARD_SPEED
TURN_SPEED = 80
CORRECTION_DIFF = 30   # steering strength when line following; must stay below the driving speed
LEFT_SPEED_MULTIPLIER = 1   # left wheel is weaker: every left wheel speed is multiplied by this

TURN_TIME = 60 / TURN_SPEED   # blind part of a junction turn (gets the sensors off the old line)

APPROACH_SPEED = 50
APPROACH_TIME = 1

CORNER_LATENCY = 60 / FORWARD_SPEED  # Time to walk forward after a corner before starting to turn to account for difference in position of wheel and sensors

# Right turns while carrying a box to a depot, outside the upper pickup area.
CARRYING_RIGHT_PRETURN_MS = 300
CARRYING_RIGHT_TURN_BLIND_MS = 400

# Depot constants
DEPOT_CORNER_LATENCY = 60 / FORWARD_SPEED  # same, but at the depot junctions
DEPOT_BACKWARDS_TIME = 35 / BACKWARD_SPEED  # back up this far after dropping off a pickup, then turn right or left straight away

# Junction detection
CONFIRM_READS = 3            # a junction must show on this many reads in a row
RECHECK_MS = 400             # junction not as expected: keep going this long, re-reading
LEG_TIMEOUT_MS = 20000       # nothing found for this long = lost
TURN_TIMEOUT_MS = 6000       # a junction turn that never finds the new line
TURN_ALIGN_SETTLE_MS = 0    # keep turning after the middle sensors confirm the new line

# Start box: line follow this long before looking for junctions (gets past the box edge)
START_FOLLOW_MS = 2000

# Turning around at the end of a branch, instead of reversing out (see docs/turnaround_test.md)
TURN_AROUND_BACK_MS = 35000 / FORWARD_SPEED            # pickups only: back up (line following) this long first
TURN_AROUND_SPEED = TURN_SPEED * 2   # spin at double speed
TURN_AROUND_BLIND_TIME = TURN_TIME * 3.6 / 2   # blind part of the spin, halved for the double speed
TURN_AROUND_TIMEOUT_MS = 30000       # keep spinning to search for the line this long before giving up

DEPOT_GAP_MS = 100000 / FORWARD_SPEED 
BETWEEN_PICKUP_BACK_MS = 25000 / FORWARD_SPEED

START_EXIT_IGNORE_MS = 2000


# Start/stop button
BUTTON_DEBOUNCE_MS = 200    # presses within this long of the last accepted one are ignored (contact bounce).
                            # Too low = one press can toggle twice; too high = a quick second press is missed
BOX_DETECT_MM = 100

# Hall sensor (DRV5055A2) for the box's magnet polarity
HALL_ZERO_READS = 50        # readings taken at startup (no magnet) to find the zero voltage (median)
HALL_THRESHOLD_V = 0.05      # change from zero that counts as a pole. Noise is ~±0.03 V; lower = picks up
                            # weaker magnets but risks noise, higher = misses weak ones (one pole was
                            # measured at only 0.15 V from zero, which this would miss)
HALL_CONFIRM_READS = 5      # same pole this many readings in a row before trusting it
HALL_READ_MS = 10           # time between readings
HALL_TIMEOUT_MS = 1000      # give up (no pole) after this long


# IR receiver (traffic light). Green is a series of lows of about 74-85 ms (measured).
IR_LOW_HOLD_MS = 60         # the pin must stay low this long to count as the green signal
IR_SAMPLE_MS = 10           # time between reads while waiting

# Amber "in operation" light: flashes for the whole mission. Red (N pole) / green (S pole)
# stay on while that box is carried. Spec: the flash rate must suit human eye perception
# (check the handout).
AMBER_FLASH_MS = 500        # half a flash cycle: 500 ms on, 500 ms off = 1 flash a second.
                            # Lower = faster flashing

# Servos (angles 0-270). PLACEHOLDERS: find the real angles by calling
# agv.hw.height_servo.set_angle(...) / agv.hw.gripper_servo.set_angle(...) from the REPL.
HEIGHT_DOWN_ANGLE = 150         # height servo: gripper lowered to box level
HEIGHT_UP_ANGLE = 130          # height servo: carrying, 20 degrees above HEIGHT_DOWN_ANGLE. Rule: the box can't be lifted more than 30 mm
GRIPPER_OPEN_ANGLE = 110        # gripper servo: jaws open, wide enough to go round the box
GRIPPER_CLOSED_ANGLE = 190     # gripper servo: holding the box. Too far may strain the servo or crush the box
SERVO_MOVE_MS = 500           # wait after each servo command so it reaches the angle before the next step. Too short = next move starts early; too long = wastes time





