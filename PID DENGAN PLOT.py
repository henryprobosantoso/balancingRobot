import time
import matplotlib.pyplot as plt
from simple_pid import PID
import RPi.GPIO as GPIO
from AngleMeterAlpha import AngleMeterAlpha


#Calling the MPU6050 data
angleMeter = AngleMeterAlpha()
angleMeter.measure()

# PID
setPoint = 0
calibrationValue = 3
P = 0
I = 0
D = 0
pid = PID(3000, 0, 0, setpoint=setPoint)

start_time = time.time()
last_time = start_time

# keep track of values for plotting
setpoint, y, x = [], [], []

while time.time() - start_time < 10:
    inputSudut = round(angleMeter.get_kalman_roll(), 3) + round(calibrationValue, 3)

    current_time = time.time()
    dt = current_time - last_time

    power = pid(inputSudut)

    x += [current_time - start_time]
    y += [inputSudut]
    setpoint += [pid.setpoint]

    if current_time - start_time > 1:
        pid.setpoint = 100

    last_time = current_time

plt.plot(x, y, label='measured')
plt.plot(x, setpoint, label='target')
plt.xlabel('time')
plt.ylabel('temperature')
plt.legend()
plt.show()