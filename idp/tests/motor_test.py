from machine import Pin
from utime import sleep

m1_dir1 = Pin(0, Pin.OUT)
m1_pwm1 = Pin(1, Pin.OUT)
m1_dir2 = Pin(4, Pin.OUT)
m1_pwm2 = Pin(5, Pin.OUT)

m2_dir1 = Pin(6, Pin.OUT)
m2_pwm1 = Pin(7, Pin.OUT)
m2_dir2 = Pin(9, Pin.OUT)
m2_pwm2 = Pin(10, Pin.OUT)

print("motors starting...")
while True:
    try:
        m1_dir1.toggle()
        sleep(1) # sleep 1sec
    except KeyboardInterrupt:
        break
m1_dir1.off()
print("Finished.")