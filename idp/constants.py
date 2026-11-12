"""Shared configuration values for AGV control."""

# Motor speed ceilings.
FORWARD_SPEED = 50
BACKWARD_SPEED = 50
TURN_SPEED = 40
CORRECTION_DIFF = 40

TURN_TIME = 2

APPROACH_SPEED = 50
APPROACH_TIME = 1

CORNER_LATENCY = 1.67 # Time to walk forward after a corner before starting to turn to account for difference in position of wheel and sensors

