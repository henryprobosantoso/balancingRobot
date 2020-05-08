import RPi.GPIO as GPIO
import os.path
import time
from simple_pid import PID
from mpu6050 import mpu6050
import math
import random
from time import sleep
from AngleMeterAlpha import AngleMeterAlpha

#Calling the MPU6050 data
angleMeter = AngleMeterAlpha()
angleMeter.measure()
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

GPIO.setup(sleepPinLeft, GPIO.OUT)
GPIO.setup(stepLeft, GPIO.OUT)
GPIO.setup(directionLeft, GPIO.OUT)

GPIO.output(sleepPinRight, GPIO.HIGH)
GPIO.output(sleepPinLeft, GPIO.HIGH)

MODE = (8,7,20,14,15,18)
GPIO.setup(MODE, GPIO.OUT)
RESOLUTION = {'full':(0,0,0,0,0,0),
                'half':(1,0,0,1,0,0),
                '1/4':(0,1,0,0,1,0),
                '1/8':(1,1,0,1,1,0),
                '1/16':(1,0,1,1,0,1)}
GPIO.output(MODE, RESOLUTION['1/8'])

def distance(a, b):
    return math.sqrt((a*a) + (b*b))
def y_rotation(x, y, z):
    radians = math.atan2(x, distance(y, z))
    return -math.degrees(radians)
def x_rotation(x, y, z):
    radians = math.atan2(y, distance(x, z))
    return math.degrees(radians)
def mundur():
    global lastState, state, mundurState
    if mundurState == 0:
        if lastState == 0:
            state = 1
        else:
            state = 0
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
    GPIO.output(directionRight, GPIO.LOW)
    GPIO.output(directionLeft, GPIO.LOW)
def maju():
    global lastState, state, mundurState
    if mundurState == 1:
        if lastState == 0:
            state = 1
        else:
            state = 0
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
    GPIO.output(directionRight, GPIO.HIGH)
    GPIO.output(directionLeft, GPIO.HIGH)
def holding():
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
def berhenti():
    GPIO.output(sleepPinRight, GPIO.LOW)
    GPIO.output(sleepPinLeft, GPIO.LOW)
def takeData():
    global gyro_total_x, gyro_total_y, last_x
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
    return last_x
# all variable
previousTimeAll = round(time.monotonic(), 4)
intervalAll = 0.01

# MPU6050 DEFINITION
setPoint = 0
previousTimeMPU6050 = round(time.monotonic(), 4)
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
# P = 1000
# I = 700
# D = 21
# speed of motor
Ps = 0
Is = 0
Ds = 0
# angle
Pa = 1500
Ia = 0
Da = 50
calibrationValue = 1
targetAngle = 0
decimalDigit = 10
decimalDigitSudut = 2
intervalMPU6050 = 0.02
step = 0
lastState = 0
mundurState = 0
motorSpeed = 0
changeResolution = 0
pidSpeed = PID(Ps, Is, Ds, setpoint = targetAngle)
pidAngle = PID(Pa, Ia, Da, setpoint = setPoint)
try:
    while True:
        currentTime = round(time.monotonic(), decimalDigit)
    # Take sudut Value
        if currentTime - previousTimeMPU6050 > intervalMPU6050:
            previousTimeMPU6050 = currentTime
#             sudut = takeData()
#             inputSudut = round(last_x, decimalDigitSudut) + round(calibrationValue, decimalDigitSudut)

            inputSudut = angleMeter.get_kalman_roll() + calibrationValue
            inputSudut = round(inputSudut, decimalDigitSudut)

            targetAngle = abs(pidSpeed(motorSpeed))
            motorSpeed = abs(pidAngle(inputSudut))
            pidSpeed.setpoint = targetAngle

            motorSpeed = round(motorSpeed, decimalDigit)
            if motorSpeed > 0:
                output = 1/motorSpeed
                output = round(output, decimalDigit)
            if inputSudut > 0:
                mundur()
                mundurState = 1
#                 print("maju")
            elif inputSudut < 0:
                maju()
                mundurState = 0
#                 print("mundur")
            else:
                holding()
#                 print("berhenti")
            print((inputSudut),",", (output), ",", (step))
#         if currentTime - previousTimeStepper > output:
#             previousTimeStepper = currentTime
#             GPIO.output(stepRight, 1)
#             GPIO.output(stepLeft, 1)
#             GPIO.output(stepRight, 0)
#             GPIO.output(stepLeft, 0)

    # Stepper control
        if currentTime - previousTimeStepper > output:
            previousTimeStepper = currentTime
            if mundurState == 1:
                step = step - 1
            else:
                step = step + 1

            if state == 0:
                state = 1
            else:
                state = 0
            GPIO.output(stepRight, state)
            GPIO.output(stepLeft, state)
            lastState = state
except KeyboardInterrupt:
    berhenti()