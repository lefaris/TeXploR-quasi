# Lauren Ervin
# Movement w/ start-up acceleration
# and cool-down decceleration for TeXploR2

from time import sleep
import RPi.GPIO as GPIO

# Shifting mass 1
DIR = 6
STEP = 5

# Shifting mass 2
DIR2 = 13
STEP2 = 12

# Clockwise/counterclockwise and steps per revolution
CW = 1
CCW = 0
SPR = 200

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(STEP2, GPIO.OUT)
GPIO.output(DIR, CW)

acceleration_count = SPR # Short time to speed up
decceleration_count = SPR # Short time to slow down
step_count = SPR*4 # Travel normal speed for most of arc
delay = 0.003 # Controls the speed of commands inversely
slow_delay = 0.01 # Acceleration/deceleration speed

# Motor 1 travel from 0 to 180
for x in range(acceleration_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(slow_delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(slow_delay)

for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)

for x in range(decceleration_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(slow_delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(slow_delay)

# Motor 2 travel from 180 to 0
sleep(0.05)
GPIO.output(DIR2, CW)
for x in range(acceleration_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(slow_delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(slow_delay)

for x in range(step_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(delay)

for x in range(decceleration_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(slow_delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(slow_delay)

# Motor 1 travel from 180 to 0
sleep(0.05)
GPIO.output(DIR, CCW)
for x in range(acceleration_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(slow_delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(slow_delay)

for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)

for x in range(decceleration_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(slow_delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(slow_delay)

# Motor 2 travel from 0 to 180
sleep(0.05)
GPIO.output(DIR2, CCW)
for x in range(acceleration_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(slow_delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(slow_delay)

for x in range(step_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(delay)

for x in range(decceleration_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(slow_delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(slow_delay)

GPIO.cleanup()