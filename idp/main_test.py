
from agv import AGV
from constants import *
from utime import sleep, sleep_ms

def timed_turn(agv, direction):
    agv.stop()
    agv.turn(direction, TURN_SPEED)
    sleep(TURN_TIME)
    agv.stop()

def continuous_turn(agv, direction):
    agv.stop()
    agv.turn(direction, TURN_SPEED)
    sleep(TURN_TIME)
    while agv.identify_config(agv.read_line_sensor()) != "line":
        #print(agv.read_line_sensor())
        pass
    agv.stop()
    return
    
def main():
    try:
        agv = AGV()
        junction_active = False

        # 1. Initial left turn
        continuous_turn(agv, "right")
        agv.state.phase = "to_corner"

        while True:
            agv.follow_line()
    except:
        agv.stop()


if __name__ == "__main__":
    main()

