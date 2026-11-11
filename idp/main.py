# main launch and run file for the project

from agv import AGV
from hardware import create_hardware
from constants import *

from utime import sleep

'''
state machine

(1) pick-up <--> (2) delivery
(differentiate between left and right, top and bottom)

task workflow
- retrieve bottom left box
- check magnet polarity
- deliver box to location depending on magnet polarity
- retrieve bottom right box
- check magnet polarity
- deliver box to location depending on magnet polarity
- retrieve top left box
- check magnet polarity
- deliver box to location depending on magnet polarity
- retrieve top right box
- check magnet polarity
- deliver box to location depending on magnet polarity

parallel tasks
- led flashing depending on operation state
- line sensor reading 

potential states for map
- bottom left
- delivery left
- bottom right
- delivery right
- top left
- top right
- start box

logic for vehicle movement
- start by going to bottom left rack ('pick-up'state)
- sense right corner, enter 'bottom left' map state
- count 2 right turns, on the third turn go right
- go until the end of the line, check vacancy eith hall sensor
- if vancant, turn around, double right to check next spot (all 6 spots)
- in all cases, drive until the end of the line and detect T shape
- when box is picked up, check polarity, enter 'delivery' state and set target (left/right station)
- deliver box to target, meanwhile changing game states and begin next pick-up

''' 


def main():
    agv = AGV(state="start")

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