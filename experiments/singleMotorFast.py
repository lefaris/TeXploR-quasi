# Lauren Ervin
# Initial test of faster movement w/
# one motor at a time for TeXploR2

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

step_count = SPR*6 # Travel full semi-circular arc
delay = 0.002 # Controls the speed of commands inversely

# Motor 1 travel from 0 to 180
for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)

# Motor 2 travel from 180 to 0
sleep(0.5)
GPIO.output(DIR2, CW)
for x in range(step_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(delay)

# Motor 1 travel from 180 to 0
sleep(0.5)
GPIO.output(DIR, CCW)
for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)

# Motor 2 travel from 0 to 180
sleep(0.5)
GPIO.output(DIR2, CCW)
for x in range(step_count):
    GPIO.output(STEP2, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP2, GPIO.LOW)
    sleep(delay)

GPIO.cleanup()