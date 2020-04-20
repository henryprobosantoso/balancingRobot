# '''HARIOMHARIBOLJAIMATAJIPITAJIKIJAIJAI'''

#a python script to be used with Balancing robots

# Importing all necessary librarys and classes
from mpu6050 import mpu6050
from time import sleep
import math
from pidcontroller import PIDController
import  RPi.GPIO as GPIO
from AngleMeterAlpha import AngleMeterAlpha



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



sensor = mpu6050(0x68)
#K and K1 --> Constants used with the complementary filter
K = 0.98
K1 = 1 - K

time_diff = 0.02
ITerm = 0

#Calling the MPU6050 data
angleMeter = AngleMeterAlpha()
angleMeter.measure()

setPoint = 0
calibrationValue = 1
#the so called 'main loop' that loops around and tells the motors wether to move or not
try:
    while True:
        inputSudut = round(angleMeter.get_kalman_roll(), 1) + calibrationValue
        #setting the PID values. Here you can change the P, I and D values according to yiur needs

    #     PID = PIDController(P=380, I=50, D=50) #HALF
        PID = PIDController(P=1700, I=0, D=1) #eight
    #     PID = PIDController(P=3000, I=0, D=0) #sixteen
        PIDx = PID.step(inputSudut)
        #if the PIDx data is lower than 0.0 than move appropriately backward
        if PIDx < setPoint:
            forward(-(PIDx))
            #StepperFor(-PIDx)
        #if the PIDx data is higher than 0.0 than move appropriately forward
        elif PIDx > setPoint:
            backward((PIDx))
            #StepperBACK(PIDx)
        #if none of the above statements is fulfilled than do not move at all
        else:
            equilibrium()
#         print(inputSudut, 'PID: ', int(PIDx))
        sleep(0.01)

except KeyboardInterrupt:
    berhenti()