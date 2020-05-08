from mpu6050 import mpu6050
from time import sleep
import math
import RPi.GPIO as GPIO
import os.path
from AngleMeterAlpha import AngleMeterAlpha
from guizero import App, Text, Slider, PushButton
import time
import pigpio
from simple_pid import PID
import matplotlib.pyplot as plt

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
#Pulse width modulation: the speed changes accordingly the inclination angle
PWM1 = GPIO.PWM(stepRight, 50)
PWM2 = GPIO.PWM(stepLeft, 50)
PWM1.start(0)
PWM2.start(0)

#PIGPIO LIBRARY
# PWM1 = pigpio.pi()
# PWM2 = pigpio.pi()
# PWM1.set_mode(stepRight, pigpio.OUTPUT)
# PWM2.set_mode(stepLeft, pigpio.OUTPUT)

#Calling the MPU6050 data
angleMeter = AngleMeterAlpha()
angleMeter.measure()



def backward(velocity):
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
    GPIO.output(directionRight, GPIO.LOW)
    GPIO.output(directionLeft, GPIO.LOW)
#     PWM1.set_PWM_dutycycle(stepRight, 128)
#     PWM2.set_PWM_dutycycle(stepLeft, 128)
#     PWM1.set_PWM_frequency(stepRight, velocity)
#     PWM2.set_PWM_frequency(stepLeft, velocity)
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
#     PWM1.set_PWM_dutycycle(stepRight, 128)
#     PWM2.set_PWM_dutycycle(stepLeft, 128)
#     PWM1.set_PWM_frequency(stepRight, velocity)
#     PWM2.set_PWM_frequency(stepLeft, velocity)
    PWM1.ChangeDutyCycle(50)
    PWM2.ChangeDutyCycle(50)
    PWM1.ChangeFrequency(velocity)
    PWM2.ChangeFrequency(velocity)

#If the PID value is 0 (the Robot is 'balanced') it uses this equilibrium function.
def equilibrium():
#     PWM1.set_PWM_dutycycle(stepRight, 0)
#     PWM2.set_PWM_dutycycle(stepLeft, 0)
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)

#If the PID value is 0 (the Robot is 'balanced') it uses this equilibrium function.
def berhenti():
    global stopMainLoop
#     PWM1.set_PWM_dutycycle(stepRight, 0)
#     PWM2.set_PWM_dutycycle(stepLeft, 0)
    PWM1.ChangeDutyCycle(0)
    PWM2.ChangeDutyCycle(0)
    GPIO.output(sleepPinRight, GPIO.LOW)
    GPIO.output(sleepPinLeft, GPIO.LOW)
    stopMainLoop = True
def mulai():
    global stopMainLoop
#     PWM1.hardware_PWM(stepRight, 100, 0)
#     PWM2.hardware_PWM(stepLeft, 100, 0)
    GPIO.output(sleepPinRight, GPIO.HIGH)
    GPIO.output(sleepPinLeft, GPIO.HIGH)
    stopMainLoop = False
# 16
PAwal = 300#3000
IAwal = 0#65.8
DAwal = 0#9
# 8
# PAwal = 1700
# IAwal = 0
# DAwal = 0
# FULL
# PAwal = 150
# IAwal = 40
# DAwal = 0
calibrationValueAwal = 1.37
batasAwal = 0
batas = 0.6
setPoint = 0
calibrationValue = 0
P = 0
I = 0
D = 0

currentTime = 0
previousTime = round(time.monotonic(), 3)
cumError = 0
lastError = 0
error = 0
elapsedTime = 0
rateError = 0

stopMainLoop = False
pid = PID(P, I, D, setpoint = 0)

def changeCalibrationValue(CalibrationSliderValue):
    global calibrationValue
    calibrationValue = calibrationValueAwal + float(CalibrationSliderValue) * 1
    print(calibrationValue)
def changeSetPointValue(setPointSliderValue):
    global batas
    batas = batasAwal + float(setPointSliderValue) * 0.01
    print(setPoint)
def changePValue(PSliderValue):
    global P
    P = PAwal + float(PSliderValue)
    print(P)
def changeIValue(ISliderValue):
    global I
    global cumError
    I = IAwal + float(ISliderValue)* 1
    cumError = 0
    print(I)
def changeDValue(DSliderValue):
    global D
    global rateError
    D = DAwal + float(DSliderValue)* 0.1
    rateError = 0
    print(D)
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

changeCalibrationValue(0)
changeSetPointValue(0)
changePValue(0)
changeIValue(0)
changeDValue(0)
decimalDigit = 6
decimalDigitSudut = 6

def mainLoop():
    if stopMainLoop == False:
        inputSudut = angleMeter.get_kalman_roll() + calibrationValue
        inputSudut = round(inputSudut, decimalDigitSudut)
        pid.tunings = (P, I, D)
        output = round(pid(inputSudut), decimalDigit)
        print((inputSudut), ",", (output))
#         output = computePID(inputSudut)
#         output = int(output)

        if inputSudut > -batas and inputSudut < batas:
            equilibrium()
        elif output > 0:
            forward(output)
        elif output < 0:
            backward(abs(output))


#         if inputSudut > 0:
#             maju = True
#         else:
#             maju = False
#         print(round(inputSudut, 1), 'PID: ', output)
#         print(stopMainLoop)
#         if output == 0:
#             equilibrium()
#             print("equi")
#         elif inputSudut < -setPoint and maju == False:
#             forward(abs(output))
#             forward(output)
#             print(inputSudut, 'output : ', output)
#             print("forward")
#         elif inputSudut > setPoint and maju == True:
#             print(inputSudut, 'output : ', output)
#             print("back")
#             backward(abs(output))
#             backward(-output)
#         else:
#             equilibrium()
#             print("equi")
    else:
        print("berhenti")


app = App(title = "PID Control Setting")
text = Text(app)
text.repeat(10, mainLoop)
welcomeMessage = Text(app, text = "PID Control Setting")

calibrationSlider = Slider(app, command = changeCalibrationValue, start=-30, end=30)
setPointSlider = Slider(app, command = changeSetPointValue, start=-30, end=30)
PSlider = Slider(app, command = changePValue, start=-1000, end=1000)
ISlider = Slider(app, command = changeIValue, start=-3000, end=3000)
DSlider = Slider(app, command = changeDValue, start=-3000, end=3000)
button1 = PushButton(app, command=berhenti)
button2 = PushButton(app, command=mulai)
app.display()