# hardware initialization and management
# initialise actual hardware

def create_hardware():
    left_motor = Motor(pin=1)
    right_motor = Motor(pin=2)

    line_sensor = LineSensor(pin=3)
    distance_sensor = DistanceSensor(pin=4)

    return Hardware(
        left_motor=left_motor,
        right_motor=right_motor,
        line_sensor=line_sensor,
        distance_sensor=distance_sensor
    )