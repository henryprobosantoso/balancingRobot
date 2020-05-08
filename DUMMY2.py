import RPi.GPIO as GPIO
import os.path
import time
from simple_pid import PID
from mpu6050 import mpu6050
import math
import random

GPIO.setmode(GPIO.BCM) #Setting the Mode to use. I am using the BCM setup
GPIO.setwarnings(False)

#Declaring the GPIO Pins that the motor controller is set with
m0Right = 8
m1Right = 7
m2Right = 20
sleepPinRight = 16
stepRight = 12
directionRight = 21

m0Left = 14
m1Left = 15
m2Left = 18
sleepPinLeft = 23
stepLeft = 13
directionLeft = 25

GPIO.setup(sleepPinRight, GPIO.OUT)
GPIO.setup(stepRight, GPIO.OUT)
GPIO.setup(directionRight, GPIO.OUT)
GPIO.setup(m0Right, GPIO.OUT)
GPIO.setup(m1Right, GPIO.OUT)
GPIO.setup(m2Right, GPIO.OUT)

GPIO.setup(sleepPinLeft, GPIO.OUT)
GPIO.setup(stepLeft, GPIO.OUT)
GPIO.setup(directionLeft, GPIO.OUT)
GPIO.setup(m0Left, GPIO.OUT)
GPIO.setup(m1Left, GPIO.OUT)
GPIO.setup(m2Left, GPIO.OUT)

GPIO.output(sleepPinRight, GPIO.HIGH)
GPIO.output(m0Right, GPIO.HIGH)
GPIO.output(m1Right, GPIO.LOW)
GPIO.output(m2Right, GPIO.LOW)
GPIO.output(sleepPinLeft, GPIO.HIGH)
GPIO.output(m0Left, GPIO.HIGH)
GPIO.output(m1Left, GPIO.LOW)
GPIO.output(m2Left, GPIO.LOW)
# LOW
def distance(a, b):
    return math.sqrt((a*a) + (b*b))
def y_rotation(x, y, z):
    radians = math.atan2(x, distance(y, z))
    return -math.degrees(radians)
def x_rotation(x, y, z):
    radians = math.atan2(y, distance(x, z))
    return math.degrees(radians)
def maju():
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
    GPIO.output(directionRight, GPIO.HIGH)
    GPIO.output(directionLeft, GPIO.HIGH)
def mundur():
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
    GPIO.output(directionRight, GPIO.LOW)
    GPIO.output(directionLeft, GPIO.LOW)
def berhenti():
    GPIO.output(sleepPinRight, GPIO.LOW)
    GPIO.output(sleepPinLeft, GPIO.LOW)

# all variable
previousTimeAll = round(time.monotonic(), 4)
intervalAll = 0.01

# MPU6050 DEFINITION
setPoint = 0
previousTimeMPU6050 = round(time.monotonic(), 4)
intervalMPU6050 = 0.01
inputSudut = 0
sensor = mpu6050(0x68)
K = 0.98
K1 = 1 - K
time_diff = 0.02
ITerm = 0

#Calling the MPU6050 data
accel_data = sensor.get_accel_data()
gyro_data = sensor.get_gyro_data()
aTempX = accel_data['x']
aTempY = accel_data['y']
aTempZ = accel_data['z']
gTempX = gyro_data['x']
gTempY = gyro_data['y']
gTempZ = gyro_data['z']
last_x = x_rotation(aTempX, aTempY, aTempZ)
last_y = y_rotation(aTempX, aTempY, aTempZ)
gyro_offset_x = gTempX
gyro_offset_y = gTempY
gyro_total_x = (last_x) - gyro_offset_x
gyro_total_y = (last_y) - gyro_offset_y

# STEPPER DEFINITION
previousTimeStepper = round(time.monotonic(), 4)
output = 1
state = 0
batas = 0.1
afterStepperCodeTime = 0
enterStepperCodeTime = 0
# PID DEFINITION
P = 50
I = 0
D = 0
calibrationValue = 1.8
pid = PID(P, I, D, setpoint = setPoint)
decimalDigit = 4
try:
    while True:
        currentTime = round(time.monotonic(), decimalDigit)
    # Take inputSudut Value
        if currentTime - previousTimeMPU6050 > intervalMPU6050:
            previousTimeMPU6050 = currentTime
            accel_data = sensor.get_accel_data()
            gyro_data = sensor.get_gyro_data()
            accelX = accel_data['x']
            accelY = accel_data['y']
            accelZ = accel_data['z']
            gyroX = gyro_data['x']
            gyroY = gyro_data['y']
            gyroZ = gyro_data['z']
            gyroX -= gyro_offset_x
            gyroY -= gyro_offset_y
            gyro_x_delta = (gyroX * time_diff)
            gyro_y_delta = (gyroY * time_diff)
            gyro_total_x += gyro_x_delta
            gyro_total_y += gyro_y_delta
            rotation_x = x_rotation(accelX, accelY, accelZ)
            rotation_y = y_rotation(accelX, accelY, accelZ)
            last_x = K * (last_x + gyro_x_delta) + (K1 * rotation_x)
            inputSudut = round(last_x, decimalDigit) + round(calibrationValue, decimalDigit)
            if inputSudut > 0:
                mundur()
#                 print("maju")
            elif inputSudut < 0:
                maju()
#                 print("mundur")
            else:
                berhenti()
#                 print("berhenti")
#         berhenti()
    # PID and print
        if currentTime - previousTimeAll > intervalAll:
            previousTimeAll = currentTime
            outputPID = abs(pid(inputSudut))
            if outputPID > 0:
                output = 1/outputPID
                output = round(output, decimalDigit)
            print(inputSudut)
    # Stepper control
        if currentTime - previousTimeStepper > output:
            previousTimeStepper = currentTime
            if state == 0:
                state = 1
            else:
                state = 0
            GPIO.output(stepRight, state)
            GPIO.output(stepLeft, state)


except KeyboardInterrupt:
    berhenti()