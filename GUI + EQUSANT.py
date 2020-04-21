from mpu6050 import mpu6050
from time import sleep
import math
from pidcontroller import PIDController
import RPi.GPIO as GPIO
from AngleMeterAlpha import AngleMeterAlpha
from guizero import App, Text, Slider

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
GPIO.output(m1Right, GPIO.HIGH)
GPIO.output(m2Right, GPIO.LOW)
GPIO.output(sleepPinLeft, GPIO.HIGH)
GPIO.output(m0Left, GPIO.HIGH)
GPIO.output(m1Left, GPIO.HIGH)
GPIO.output(m2Left, GPIO.LOW)

#Pulse width modulation: the speed changes accordingly the inclination angle
PWM1 = GPIO.PWM(stepRight, 50)
PWM2 = GPIO.PWM(stepLeft, 50)

PWM1.start(0)
PWM2.start(0)

#Calling the MPU6050 data
angleMeter = AngleMeterAlpha()
angleMeter.measure()


def changeCalibrationValue(CalibrationSliderValue):
    global calibrationValue
    calibrationValue = calibrationValueAwal + float(CalibrationSliderValue) * 0.1
    print(calibrationValue)
def changeSetPointValue(setPointSliderValue):
    global setPoint
    setPoint = setPointAwal + float(setPointSliderValue) * 0.1
    print(setPoint)
def changePValue(PSliderValue):
    global P
    P = PAwal + float(PSliderValue)
    print(P)
def changeIValue(ISliderValue):
    global I
    I = IAwal + float(ISliderValue)
    print(I)
def changeDValue(DSliderValue):
    global D
    D = DAwal + float(DSliderValue)
    print(D)

def counter():
    print("calibrationValue = ", calibrationValue)
    print("P = ", P)
    print("I = ", I)
    print("D = ", D)
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

PAwal = 1700
IAwal = 0
DAwal = 0
calibrationValueAwal = 0.5
setPointAwal = 0.1

setPoint = 0
calibrationValue = 0
P = 0
I = 0
D = 0

def mainLoop():
    inputSudut = round(angleMeter.get_kalman_roll(), 1) + round(calibrationValue, 1)
    PID = PIDController(P, I, D) #eight
    PIDx = round(PID.step(inputSudut), 1)
    if inputSudut < -setPoint:
        forward(-(PIDx))
#         print("forward")
    elif inputSudut > setPoint:
        backward((PIDx))
#         print("back")
    else:
        equilibrium()
#         print("equi")
    print(round(inputSudut, 2), 'PID: ', PIDx)
#     counter()
changeCalibrationValue(0)
changePValue(0)
changeIValue(0)
changeDValue(0)

app = App(title = "PID Control Setting")
text = Text(app, text = 1)
text.repeat(1, mainLoop)
welcomeMessage = Text(app, text = "PID Control Setting")

calibrationSlider = Slider(app, command = changeCalibrationValue, start=-30, end=30)
setPointSlider = Slider(app, command = changeSetPointValue, start=-30, end=30)
PSlider = Slider(app, command = changePValue, start=-1000, end=1000)
ISlider = Slider(app, command = changeIValue, start=-3000, end=3000)
DSlider = Slider(app, command = changeDValue, start=-3000, end=3000)
app.display()