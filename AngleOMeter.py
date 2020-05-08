import time
from AngleMeterAlpha import AngleMeterAlpha

angleMeter = AngleMeterAlpha()
angleMeter.measure()

while True:
    print(angleMeter.get_kalman_roll(),",", angleMeter.get_complementary_roll())
    #print(angleMeter.get_int_roll(), angleMeter.get_int_pitch())
    time.sleep(0.01)