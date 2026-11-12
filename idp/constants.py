"""Shared configuration values for AGV control."""

# Motor speed ceilings.
FORWARD_SPEED = 100
BACKWARD_SPEED = 100
TURN_SPEED = 80
CORRECTION_DIFF = 20

TURN_TIME = 0.5
APPROACH_SPEED = 50
APPROACH_TIME = 1

CORNER_LATENCY = 0.5 # Time to walk forward after a corner before starting to turn to account for difference in position of wheel and sensors