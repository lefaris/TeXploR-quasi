# Lauren Ervin
# Movement w/ dual motor driving
# simultaneously for sustaining 
# momentum w/ TeXploR2

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

transition_count = SPR # Short time to speed up
step_count = SPR*5 # Travel normal speed for most of arc
delay = 0.004 # Controls the speed of commands inversely
fast_delay = 0.002 # Faster transition speed

# Motor 1 travel from 0 to 180 while 
# motor 2 moves from 180 to 150
for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)

for x in range(transition_count):
    GPIO.output(DIR, CW)
    GPIO.output(STEP, GPIO.HIGH)
    sleep(fast_delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(fast_delay)
    GPIO.output(DIR2, CW)
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(fast_delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(fast_delay)
    

# Motor 2 continues from 150 to 0
# while motor 1 moves from 180 to 150
sleep(0.05)
GPIO.output(DIR2, CW)
for x in range(step_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(delay)

for x in range(transition_count):
    GPIO.output(DIR2, CW)
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(fast_delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(fast_delay)
    GPIO.output(DIR, CCW)
    GPIO.output(STEP, GPIO.HIGH)
    sleep(fast_delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(fast_delay)

# Motor 1 continues from 150 to 0
# while motor 2 moves from 0 to 30
sleep(0.05)
GPIO.output(DIR, CCW)
for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)

for x in range(transition_count):
    GPIO.output(DIR, CCW)
    GPIO.output(STEP, GPIO.HIGH)
    sleep(fast_delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(fast_delay)
    GPIO.output(DIR2, CCW)
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(fast_delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(fast_delay)

# Motor 2 continues from 30 to 180
sleep(0.05)
GPIO.output(DIR2, CCW)
for x in range(step_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(delay)

for x in range(transition_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(delay)

GPIO.cleanup()