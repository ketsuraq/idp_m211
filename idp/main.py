
from agv import AGV
from constants import *
from utime import sleep, sleep_ms

def timed_turn(agv, direction):
    agv.stop()
    agv.turn(direction, TURN_SPEED)
    sleep(TURN_TIME)
    agv.stop()


def main():
    agv = AGV()
    junction_active = False

    # 1. Initial left turn
    timed_turn(agv, "left")
    agv.state.phase = "to_corner"

    while True:
        config = agv.follow_line(FORWARD_SPEED)

        junction = config in (
            "left turn", "right turn", "parallel"
        )

        # Detect each junction only once
        if junction and not junction_active:
            junction_active = True

            action = agv.state.handle_junction(config)

            if action in ("left", "right"):
                timed_turn(agv, action)

            elif action == "stop":
                print("Unexpected junction")
                break

        elif not junction:
            junction_active = False

        # Third junction reached
        if agv.state.phase == "to_end":
            break

        if config in ("line lost", "unknown"):
            agv.stop()
            print("Line lost or unknown:", config)
            break

        sleep_ms(10)

    # 2. Follow final branch
    if agv.state.phase == "to_end":
        print("Following final branch")

        while True:
            config = agv.follow_line(FORWARD_SPEED)

            if config == "line lost":
                agv.stop()
                print("End of line detected")
                break

            elif config == "unknown":
                agv.stop()
                print("Unknown sensor pattern")
                break

            sleep_ms(10)


    agv.stop()


if __name__ == "__main__":
    main()
