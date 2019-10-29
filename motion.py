import RPi.GPIO as gpio
import time

LEFT = 16
RIGHT = 18
MOTION = 23

gpio.setmode(gpio.BCM)
gpio.setup(LEFT, gpio.OUT)
gpio.setup(RIGHT, gpio.OUT)
gpio.setup(MOTION, gpio.IN)
l = gpio.PWM(LEFT, 50)
r = gpio.PWM(RIGHT, 50)
r.start(0)
l.start(0)

try:
    while True :
        if gpio.input(MOTION) == gpio.HIGH:
            print("MOTION DETECTED")
            r.ChangeDutyCycle(5)
            l.ChangeDutyCycle(10)

except KeyboardInterrupt :
    l.stop()
    r.stop()
    gpio.cleanup()
