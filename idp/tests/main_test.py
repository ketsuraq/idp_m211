
from agv import AGV
from constants import *
from utime import sleep, sleep_ms, ticks_ms, ticks_diff

JUNCTIONS = {"left turn": "left", "right turn": "right"}


def wait_for_junction(agv):
    """Line follow until a left/right junction is seen twice in a row; return its direction."""
    last = None
    while True:
        config = agv.follow_line(FORWARD_SPEED)

        if config in JUNCTIONS and config == last:
            return JUNCTIONS[config]

        if config == "parallel":
            raise RuntimeError("T-junction: direction is ambiguous")
        if config == "line lost":
            raise RuntimeError("Line lost before reaching a junction")

        last = config
        sleep_ms(10)


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


def follow_until_end(agv):
    """Line follow (self-correcting) until the line is lost or another junction appears."""
    while True:
        config = agv.follow_line(FORWARD_SPEED)
        if config in ("line lost", "parallel") or config in JUNCTIONS:
            print("Stopping at:", config)
            return
        sleep_ms(10)


def main():
    agv = AGV()
    try:
        direction = wait_for_junction(agv)
        print("Junction found, turning", direction)

        turn_at_junction(agv, direction)
        print("Turn done, following line")

        follow_until_end(agv)
    except KeyboardInterrupt:
        print("Ctrl-C")
        agv.stop()
    except Exception as e:
        print("Error:", e)
    finally:
        agv.stop()


if __name__ == "__main__":
    main()
