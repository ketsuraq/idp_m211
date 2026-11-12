"""Shared configuration values for AGV control."""

# Motor speed ceilings.
FORWARD_SPEED = 20
BACKWARD_SPEED = 20
TURN_SPEED = 40
CORRECTION_DIFF = 50

TURN_TIME = 0.3
APPROACH_SPEED = 50
APPROACH_TIME = 1

CORNER_LATENCY = 0.5 # Time to walk forward after a corner before starting to turn to account for difference in position of wheel and sensors
