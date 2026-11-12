
from agv import AGV
from constants import *
from utime import sleep, sleep_ms, ticks_ms, ticks_diff

test_speed = 100
max_run_ms = 10000


OK_CONFIGS = ("line", "left correction", "right correction")
agv = AGV()
reason = "timeout"

try:
    #countdown for putting on track
    print("Put robot on line. start in:")
    for seconds in range(3,0,-1):
        print(seconds)
        sleep(1)

    last = None
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start)< max_run_ms:
        config = agv.follow_line(test_speed)

        if config != last:
            print("config:", config)
            last = config

        if config not in OK_CONFIGS:
            reason = config
            break

        sleep_ms(10)

except KeyboardInterrupt:
    reason = "Ctrl-C"
finally:
    agv.stop()
    print("Stopped. reason:", reason)


