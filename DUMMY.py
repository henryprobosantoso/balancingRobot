'''HARIOMHARIBOLJAIMATAJIPITAJIKIJAIJAI'''

#a python script to be used with Balancing robots

# Importing all necessary librarys and classes
from mpu6050 import mpu6050
from time import sleep
import math
from pidcontroller import PIDController
import  RPi.GPIO as GPIO
import time
from simple_pid import PID
from AngleMeterAlpha import AngleMeterAlpha

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
GPIO.output(m0Right, GPIO.LOW)
GPIO.output(m1Right, GPIO.HIGH)
GPIO.output(m2Right, GPIO.LOW)
GPIO.output(sleepPinLeft, GPIO.HIGH)
GPIO.output(m0Left, GPIO.LOW)
GPIO.output(m1Left, GPIO.HIGH)
GPIO.output(m2Left, GPIO.LOW)
# LOW
PWM1 = GPIO.PWM(stepRight, 50)
PWM2 = GPIO.PWM(stepLeft, 50)
PWM1.start(0)
PWM2.start(0)

def backward(velocity):
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
    GPIO.output(directionRight, GPIO.LOW)
    GPIO.output(directionLeft, GPIO.LOW)
    PWM1.ChangeDutyCycle(50)
    PWM2.ChangeDutyCycle(50)
    PWM1.ChangeFrequency(velocity)
    PWM2.ChangeFrequency(velocity)

#Alike the backward funtion this forward function does the same thing but moves both the motors forward.
def forward(velocity):
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
    GPIO.output(directionRight, GPIO.HIGH)
    GPIO.output(directionLeft, GPIO.HIGH)
    PWM1.ChangeDutyCycle(50)
    PWM2.ChangeDutyCycle(50)
    PWM1.ChangeFrequency(velocity)
    PWM2.ChangeFrequency(velocity)

#If the PID value is 0 (the Robot is 'balanced') it uses this equilibrium function.
def equilibrium():
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)

#If the PID value is 0 (the Robot is 'balanced') it uses this equilibrium function.
def berhenti():
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    GPIO.output(sleepPinRight, GPIO.LOW)
    GPIO.output(sleepPinLeft, GPIO.LOW)


sensor = mpu6050(0x68)
#K and K1 --> Constants used with the complementary filter
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

currentTime = 0
previousTime = round(time.monotonic(), 3)
cumError = 0
lastError = 0
error = 0
elapsedTime = 0
rateError = 0
def computePID(input):
    global currentTime
    global previousTime
    global cumError
    global lastError
    global error
    global elapsedTime
    global rateError

    currentTime = round(time.monotonic(), 3)
    elapsedTime = currentTime - previousTime
    elapsedTime = round(elapsedTime, 3)

    error = setPoint - input
    cumError += error * elapsedTime
    cumError = round(cumError, 3)

    rateError = (error - lastError)/elapsedTime
    rateError = round(rateError, 3)

    out = P*error + I*cumError + D*rateError
    out = round(out, 3)

    lastError = error
    previousTime = currentTime
#     print(out)

    return out
#some math
def distance(a, b):
    return math.sqrt((a*a) + (b*b))

def y_rotation(x, y, z):
    radians = math.atan2(x, distance(y, z))
    return -math.degrees(radians)

def x_rotation(x, y, z):
    radians = math.atan2(y, distance(x, z))
    return math.degrees(radians)


last_x = x_rotation(aTempX, aTempY, aTempZ)
last_y = y_rotation(aTempX, aTempY, aTempZ)

gyro_offset_x = gTempX
gyro_offset_y = gTempY

gyro_total_x = (last_x) - gyro_offset_x
gyro_total_y = (last_y) - gyro_offset_y

# PID DEFINITION
setPoint = 0
calibrationValue = 2
P = 400
I = 0
D = 0
# P = 20
# I = 0
# D = 0
pid = PID(P, I, D, setpoint = setPoint)
batas = 0
decimalDigit = 0
decimalDigitSudut = 0
#the so called 'main loop' that loops around and tells the motors wether to move or not
try:
    while True:
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
        inputSudut = round(last_x, decimalDigitSudut) + round(calibrationValue, decimalDigitSudut)
        inputSudut = round(inputSudut, decimalDigit)
#         pid.tunings = (P, I, D)
#         PIDx = pid(inputSudut)
        output = computePID(inputSudut)
        PIDx = output
        if inputSudut == batas:
            equilibrium()
#             print("berhenti")
        elif PIDx < 0.0:
            backward(-float(PIDx))
#             print("back")
            #StepperFor(-PIDx)
        #if the PIDx data is higher than 0.0 than move appropriately forward
        elif PIDx > 0.0:
            forward(float(PIDx))
#             print("forw")
            #StepperBACK(PIDx)

        print(inputSudut, '|||', PIDx)
        sleep(0.02)

except KeyboardInterrupt:
    berhenti()