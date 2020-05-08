from time import sleep
import math
from pidcontroller import PIDController
import time
import PID
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

sampleTime = 0.01
lastTime = round(time.monotonic(), 3) - sampleTime
lastInput = 0
outputSum = 0
output = 0
def computePID2(input):
    global lastTime, lastInput, outputSum, output
    currentTime = round(time.monotonic(), 3)
#     print(currentTime, "|||", lastTime)

    if currentTime - lastTime >= sampleTime:
        error = setPoint - input
        dInput = (input - lastInput)
        outputSum += (I * error)

        output = P * error
        output += outputSum - D * dInput

        print(outputSum, "|||", output)
        lastInput = input
        lastTime = currentTime
    return output
# Pa = 800
# Ia = 1000
# Da = 100
P = 890
I = 10
D = 2
setPoint = 0
batas = 0
decimalDigit = 8
decimalDigitSudut = 1
calibrationValue = 1.0
ms8()
pid = PID.PID(P, I, D)
pid.SetPoint=0.0
pid.setSampleTime(0.01)

#the so called 'main loop' that loops around and tells the motors wether to move or not
try:
    while True:
        inputSudut = angleMeter.get_kalman_roll() + calibrationValue
        inputSuduty = round(inputSudut, decimalDigitSudut)
        pid.update(inputSuduty)
        output = pid.output
        PIDy = round(output, decimalDigit)
        if (inputSuduty <= batas and inputSuduty >= -batas) or PIDy == 0:
            equilibrium()
        elif PIDy < 0.0:
            backward(-int(PIDy))
        elif PIDy > 0.0:
            forward(int(PIDy))

#         print(inputSuduty, '|||', PIDy)
        sleep(0.01)

except KeyboardInterrupt:
    berhenti()