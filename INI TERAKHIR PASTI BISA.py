from time import sleep
import math
from pidcontroller import PIDController
import time
from simple_pid import PID
import pigpio
from AngleMeterAlpha import AngleMeterAlpha
from mpu6050 import mpu6050

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
def ms4():
    pi.write(sleepPinRight, 1)
    pi.write(m0Right, 0)
    pi.write(m1Right, 1)
    pi.write(m2Right, 0)
    pi.write(sleepPinLeft, 1)
    pi.write(m0Left, 0)
    pi.write(m1Left, 1)
    pi.write(m2Left, 0)
def ms8():
    pi.write(sleepPinRight, 1)
    pi.write(m0Right, 1)
    pi.write(m1Right, 1)
    pi.write(m2Right, 0)
    pi.write(sleepPinLeft, 1)
    pi.write(m0Left, 1)
    pi.write(m1Left, 1)
    pi.write(m2Left, 0)
def ms16():
    pi.write(sleepPinRight, 1)
    pi.write(m0Right, 1)
    pi.write(m1Right, 1)
    pi.write(m2Right, 1)
    pi.write(sleepPinLeft, 1)
    pi.write(m0Left, 1)
    pi.write(m1Left, 1)
    pi.write(m2Left, 1)

pi.set_PWM_dutycycle(stepRight, 128)
pi.set_PWM_dutycycle(stepLeft, 128)

#Calling the MPU6050 data
angleMeter = AngleMeterAlpha()
angleMeter.measure()

def distance(a, b):
    return math.sqrt((a*a) + (b*b))
def y_rotation(x, y, z):
    radians = math.atan2(x, distance(y, z))
    return -math.degrees(radians)
def x_rotation(x, y, z):
    radians = math.atan2(y, distance(x, z))
    return math.degrees(radians)
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
sensor = mpu6050(0x68)
K = 0.98
K1 = 1 - K
time_diff = 0.02
ITerm = 0
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
def backward(velocity):
    pi.write(sleepPinRight, 1)
    pi.write(sleepPinLeft, 1)
    pi.write(directionRight, 0)
    pi.write(directionLeft, 0)
    pi.set_PWM_dutycycle(stepRight, 128)
    pi.set_PWM_dutycycle(stepLeft, 128)
    pi.set_PWM_frequency(stepRight,velocity)
    pi.set_PWM_frequency(stepLeft,velocity)

#Alike the backward funtion this forward function does the same thing but moves both the motors forward.
def forward(velocity):
    pi.write(sleepPinRight, 1)
    pi.write(sleepPinLeft, 1)
    pi.write(directionRight, 1)
    pi.write(directionLeft, 1)
    pi.set_PWM_dutycycle(stepRight, 128)
    pi.set_PWM_dutycycle(stepLeft, 128)
    pi.set_PWM_frequency(stepRight,velocity)
    pi.set_PWM_frequency(stepLeft,velocity)

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

currentTimex = 0
previousTimex = round(time.monotonic(), 3)
cumErrorx = 0
lastErrorx = 0
errorx = 0
elapsedTimex = 0
rateErrorx = 0
currentTimey = 0
previousTimey = round(time.monotonic(), 3)
cumErrory = 0
lastErrory = 0
errory = 0
elapsedTimey = 0
rateErrory = 0
def computePIDx(input):
    global currentTimex
    global previousTimex
    global cumErrorx
    global lastErrorx
    global errorx
    global elapsedTimex
    global rateErrorx

    currentTimex = round(time.monotonic(), 3)
    elapsedTimex = currentTimex - previousTimex
    elapsedTimex = round(elapsedTimex, 3)

    errorx = setPoint - input
    cumErrorx += errorx * 0.015
    cumErrorx = round(cumErrorx, 3)

    rateErrorx = (errorx - lastErrorx)/0.015
    rateErrorx = round(rateErrorx, 3)

    outx = Pa*errorx + Ia*cumErrorx + Da*rateErrorx
    outx = round(outx, 3)

    lastErrorx = errorx
    previousTimex = currentTimex

    return outx
def computePIDy(input):
    global currentTimey
    global previousTimey
    global cumErrory
    global lastErrory
    global errory
    global elapsedTimey
    global rateErrory

    currentTimey = round(time.monotonic(), 3)
    elapsedTimey = currentTimey - previousTimey
    elapsedTimey = round(elapsedTimey, 3)

    errory = setPoint - input
    cumErrory += errory * 0.015
    cumErrory = round(cumErrory, 3)

    rateErrory = (errory - lastErrory)/0.015
    rateErrory = round(rateErrory, 3)

    outy = Pa*errory + Ia*cumErrory + Da*rateErrory
    outy = round(outy, 3)

    lastErrory = errory
    previousTimey = currentTimey

    return outy

# PID DEFINITION
# P = 300
# I = 350
# D = 2
# P = 20
# I = 0
# D = 0
# pid = PID(P, I, D, setpoint = setPoint)

# speed of motor
Ps = 0
Is = 0
Ds = 0
# angle
# Pa = 360
# Ia = 300
# Da = 5
# 1/4
# Pa = 500
# Ia = 400
# Da = 9
# 1/8
Pa = 800
Ia = 1000
Da = 100
setPoint = 0
pidSpeed = PID(Ps, Is, Ds, setpoint = 0)
pidAngle = PID(Pa, Ia, Da, setpoint = 0)
PIDx = 0
batas = 0
decimalDigit = 8
decimalDigitSudut = 8
calibrationValue = 0.39
ms8()
#the so called 'main loop' that loops around and tells the motors wether to move or not
try:
    while True:
        inputSudutx = takeData()
        inputSudutx = inputSudutx + calibrationValue
        inputSudutx = round(inputSudutx, decimalDigitSudut)
#         pidSpeed.tunings = (Ps, Is, Ds)
#         pidAngle.tunings = (Pa, Ia, Da)
#         PIDx = round(pidAngle(inputSudutx), decimalDigit)
        PIDx = computePIDx(inputSudutx)
        PIDx = round(PIDx, decimalDigit)

        inputSuduty = angleMeter.get_kalman_roll() + calibrationValue
        inputSuduty = round(inputSuduty, decimalDigitSudut)
#         pidSpeed.tunings = (Ps, Is, Ds)
#         pidAngle.tunings = (Pa, Ia, Da)
#         PIDy = round(pidAngle(inputSuduty), decimalDigit)
        PIDy = computePIDy(inputSuduty)
        PIDy = round(PIDy, decimalDigit)
        if (inputSuduty <= batas and inputSuduty >= -batas) or PIDx == 0:
            equilibrium()
        elif PIDx < 0.0:
            backward(-int(PIDx))
        elif PIDx > 0.0:
            forward(int(PIDx))

#         print(inputSudutx, '|||', PIDx)
#         print(inputSuduty, '|||', PIDy)
        print(inputSudutx, '|||', PIDx, '|||',  inputSuduty, '|||', PIDy)
        sleep(0.05)

except KeyboardInterrupt:
    berhenti()