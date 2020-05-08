from mpu6050 import mpu6050
from time import sleep
import math
from pidcontroller import PIDController
import time
from simple_pid import PID
import pigpio

pi = pigpio.pi()

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

pi.set_mode(sleepPinRight, pigpio.OUTPUT)
pi.set_mode(stepRight, pigpio.OUTPUT)
pi.set_mode(directionRight, pigpio.OUTPUT)
pi.set_mode(m0Right, pigpio.OUTPUT)
pi.set_mode(m1Right, pigpio.OUTPUT)
pi.set_mode(m2Right, pigpio.OUTPUT)

pi.set_mode(sleepPinLeft, pigpio.OUTPUT)
pi.set_mode(stepLeft, pigpio.OUTPUT)
pi.set_mode(directionLeft, pigpio.OUTPUT)
pi.set_mode(m0Left, pigpio.OUTPUT)
pi.set_mode(m1Left, pigpio.OUTPUT)
pi.set_mode(m2Left, pigpio.OUTPUT)

pi.write(sleepPinRight, 1)
pi.write(m0Right, 0)
pi.write(m1Right, 1)
pi.write(m2Right, 0)

pi.write(sleepPinLeft, 1)
pi.write(m0Left, 0)
pi.write(m1Left, 1)
pi.write(m2Left, 0)
# LOW
pi.set_PWM_dutycycle(stepRight, 128)
pi.set_PWM_dutycycle(stepLeft, 128)

def backward(velocity):
    pi.write(sleepPinRight, 1)
    pi.write(sleepPinLeft, 1)
    pi.write(directionRight, 0)
    pi.write(directionLeft, 0)
    pi.hardware_PWM(stepRight, velocity, 500000)
    pi.hardware_PWM(stepLeft, velocity, 500000)

#Alike the backward funtion this forward function does the same thing but moves both the motors forward.
def forward(velocity):
    pi.write(sleepPinRight, 1)
    pi.write(sleepPinLeft, 1)
    pi.write(directionRight, 1)
    pi.write(directionLeft, 1)
    pi.hardware_PWM(stepRight, velocity, 500000)
    pi.hardware_PWM(stepLeft, velocity, 500000)

#If the PID value is 0 (the Robot is 'balanced') it uses this equilibrium function.
def equilibrium():
    pi.set_PWM_dutycycle(stepRight, 0)
    pi.set_PWM_dutycycle(stepLeft, 0)
    pi.write(sleepPinRight, 1)
    pi.write(sleepPinLeft, 1)

#If the PID value is 0 (the Robot is 'balanced') it uses this equilibrium function.
def berhenti():
    pi.set_PWM_dutycycle(stepRight, 0)
    pi.set_PWM_dutycycle(stepLeft, 0)
    pi.write(sleepPinRight, 0)
    pi.write(sleepPinLeft, 0)


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
calibrationValue = 0
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
        pid.tunings = (P, I, D)
        PIDx = pid(inputSudut)
#         output = computePID(inputSudut)
#         PIDx = output
        if inputSudut == batas:
            equilibrium()
#             print("berhenti")
        elif PIDx < 0.0:
            backward(-int(PIDx))
#             print("back")
            #StepperFor(-PIDx)
        #if the PIDx data is higher than 0.0 than move appropriately forward
        elif PIDx > 0.0:
            forward(int(PIDx))
#             print("forw")
            #StepperBACK(PIDx)

        print(inputSudut, '|||', PIDx)
        sleep(0.02)

except KeyboardInterrupt:
    berhenti()