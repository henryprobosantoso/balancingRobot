from mpu6050 import mpu6050
from time import sleep
import math
from pidcontroller import PIDController
import  RPi.GPIO as GPIO
from AngleMeterAlpha import AngleMeterAlpha
from simple_pid import PID

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

MODE = (8, 7,20, 14, 15, 18)
GPIO.setup(MODE, GPIO.OUT)
RESOLUTION = {'full':(0,0,0,0,0,0),
                'half':(1,0,0,1,0,0),
                '1/4':(0,1,0,0,1,0),
                '1/8':(1,1,0,1,1,0),
                '1/16':(1,0,1,1,0,1)}
GPIO.output(MODE, RESOLUTION['1/4'])

#Pulse width modulation: the speed changes accordingly the inclination angle
PWM1 = GPIO.PWM(stepRight, 50)
PWM2 = GPIO.PWM(stepLeft, 50)
PWM1.start(0)
PWM2.start(0)

#This backward function takes a velocity argument that is the PID value. Both motors drives backward
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

#Calling the MPU6050 data
angleMeter = AngleMeterAlpha()
angleMeter.measure()

P = 500
I = 0
D = 0
batas = 0
setPoint = 0
calibrationValue = 0.6
decimalDigit = 6
decimalDigitSudut = 6
pid = PID(P, I, D, setpoint = 0)
#the so called 'main loop' that loops around and tells the motors wether to move or not
try:
    while True:
        inputSudut = angleMeter.get_kalman_roll() + calibrationValue
        inputSudut = round(inputSudut, decimalDigitSudut)
        #setting the PID values. Here you can change the P, I and D values according to yiur needs

        pid.tunings = (P, I, D)
        PIDx = round(pid(inputSudut), decimalDigit)
#         PID = PIDController(P=500, I=0, D=0) #eight
#         PIDx = PID.step(inputSudut)
        if inputSudut > 0:
            maju = True
        else:
            maju = False
        print(inputSudut, 'PID: ', PIDx)
#         print(stopMainLoop)
        if inputSudut < batas and inputSudut > -batas or PIDx == 0:
            equilibrium()
#             print("equi")
        elif inputSudut < -setPoint and maju == False:
            forward(abs(PIDx))
#             forward(output)
#             print(inputSudut, 'output : ', output)
#             print("forward")
        elif inputSudut > setPoint and maju == True:
#             print(inputSudut, 'output : ', output)
#             print("back")
            backward(abs(PIDx))
        else:
            equilibrium()
#             print("equi")
        sleep(0.01)

except KeyboardInterrupt:
    berhenti()