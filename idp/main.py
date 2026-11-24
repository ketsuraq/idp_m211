"""First competition run (see the docstring at the top of competition_1.py for what the
robot does). Start in the start box, facing the ramp, and press the button.

main.py runs by itself when the Pico powers up, so the countdown gives time to let go.
"""

from agv import AGV
from competition_1 import CompetitionMission, run
from utime import sleep, sleep_ms


def main():
    agv = AGV()
    agv.stop()


    while True:
        print("Press button to START")

        # Wait until the button is pressed
        while not agv.button_clicked():
            sleep_ms(20)

        print("Starting mission")

        # Create a fresh mission every time (this also starts its clock)
        mission = CompetitionMission()

        agv.mission_running = True   # run() starts the amber flashing

        try:
            run(agv, mission)

        except KeyboardInterrupt:
            print("Mission stopped by button")

        except Exception as e:
            print("Error:", e)

        finally:
            # Always stop motors, and switch every LED off
            agv.stop()
            agv.stop_mission_lights()   # run() already does this; kept in case run() never started
            agv.mission_running = False

            print("Finished in state", mission.state)
            print("Delivered:", mission.delivered)

        # Don't allow a restart until the button is released
        while agv.hw.button.is_pressed():
            sleep_ms(20)

if __name__ == "__main__":
    main()
