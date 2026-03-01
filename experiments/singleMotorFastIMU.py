# Lauren Ervin
# Faster movement w/ one motor at 
# a time for TeXploR2 w/ 1 IMU

from time import sleep
import datetime
import csv
import board
import busio
from adafruit_bno055 import BNO055_I2C
import RPi.GPIO as GPIO
import os

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

i2c = busio.I2C(board.SCL, board.SDA)
bno1 = BNO055_I2C(i2c)

# Define the filename - make sure to update for each experiment!
filename = 'bno_TeXploR_test.csv'
fieldnames = ['Timestamp', 'Euler_Roll', 'Euler_Pitch', 'Euler_Yaw', 'Acceleration_X', 'Acceleration_Y', 'Acceleration_Z']

# Write CSV header
def write_header(filename, fieldnames):
    file_exists = os.path.exists(filename) and os.stat(filename).st_size != 0
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()

# Append BNO data
def log_bno_data(filename, fieldnames):
    roll, pitch, yaw = bno.euler
    accel_x, accel_y, accel_z = bno.acceleration
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    data = {
        'Timestamp': timestamp,
        'Euler_Roll': roll,
        'Euler_Pitch': pitch,
        'Euler_Yaw': yaw,
        'Acceleration_X': accel_x,
        'Acceleration_Y': accel_y,
        'Acceleration_Z': accel_z
    }

    with open(filename, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writerow(data)
            
write_header(filename, fieldnames) # Generate header

# Motor 1 travel from 0 to 180
for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    log_bno_data(filename, fieldnames)
    sleep(delay)
    log_bno_data(filename, fieldnames)
    GPIO.output(STEP, GPIO.LOW)
    log_bno_data(filename, fieldnames)
    sleep(delay)
    log_bno_data(filename, fieldnames)

# Motor 2 travel from 180 to 0
sleep(0.5)
GPIO.output(DIR2, CW)
for x in range(step_count):
    GPIO.output(STEP2, GPIO.HIGH)
    log_bno_data(filename, fieldnames)
    sleep(delay)
    log_bno_data(filename, fieldnames)
    GPIO.output(STEP2, GPIO.LOW)
    log_bno_data(filename, fieldnames)
    sleep(delay)
    log_bno_data(filename, fieldnames)

# Motor 1 travel from 180 to 0
sleep(0.5)
GPIO.output(DIR, CCW)
for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    log_bno_data(filename, fieldnames)
    sleep(delay)
    log_bno_data(filename, fieldnames)
    GPIO.output(STEP, GPIO.LOW)
    log_bno_data(filename, fieldnames)
    sleep(delay)
    log_bno_data(filename, fieldnames)

# Motor 2 travel from 0 to 180
sleep(0.5)
GPIO.output(DIR2, CCW)
for x in range(step_count):
    GPIO.output(STEP2, GPIO.HIGH)
    log_bno_data(filename, fieldnames)
    sleep(delay)
    log_bno_data(filename, fieldnames)
    GPIO.output(STEP2, GPIO.LOW)
    log_bno_data(filename, fieldnames)
    sleep(delay)
    log_bno_data(filename, fieldnames)

GPIO.cleanup()
