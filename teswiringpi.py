import RPi.GPIO as GPIO
import wiringpi


GPIO.setmode(GPIO.BCM) #Setting the Mode to use. I am using the BCM setup
GPIO.setwarnings(False)

#Declaring the GPIO Pins that the motor controller is set with
m0Right = 8
m1Right = 7
m2Right = 20
sleepPinRight = 16
stepRight = 12
directionRight = 21
GPIO.setup(sleepPinRight, GPIO.OUT)
# GPIO.setup(stepRight, GPIO.OUT)
GPIO.setup(directionRight, GPIO.OUT)
GPIO.setup(m0Right, GPIO.OUT)
GPIO.setup(m1Right, GPIO.OUT)
GPIO.setup(m2Right, GPIO.OUT)
# PWM1 = GPIO.PWM(stepRight, 50)
# PWM1.start(0)

wiringpi.wiringPiSetup()
wiringpi.softToneCreate(stepRight)

try:
    while True:
        wiringpi.softToneWrite(stepRight, 100)


except KeyboardInterrupt:
    berhenti()