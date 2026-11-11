"""Launch the AGV and run the current motor-control demonstration."""

from agv import AGV
from constants import BACKWARD_SPEED, FORWARD_SPEED, TURN_SPEED

from utime import sleep


def main():
    """Run a short manual motion test using an AGV with its initial RobotState."""
    agv = AGV()

    agv.forward(speed=FORWARD_SPEED)
    sleep(2)
    agv.stop()
    sleep(1)

    agv.backward(speed=BACKWARD_SPEED)
    sleep(2)
    agv.stop()
    sleep(1)

    agv.turn(direction="left", speed=TURN_SPEED)
    sleep(2)
    agv.stop()
    sleep(1)
    agv.turn(direction="right", speed=TURN_SPEED)
    sleep(2)
    agv.stop()
    sleep(1)
    


if __name__ == "__main__":
    main()
