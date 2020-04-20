import time
from AngleMeterAlpha import AngleMeterAlpha

angleMeter = AngleMeterAlpha()
angleMeter.measure()

while True:
    x = round(angleMeter.get_kalman_roll(), 2)
#     print(angleMeter.get_kalman_roll(),",", angleMeter.get_complementary_roll(), ",",angleMeter.get_kalman_pitch(),",", angleMeter.get_complementary_pitch(),".")
    print(x)
#     print(angleMeter.get_int_roll(), angleMeter.get_int_pitch())
    time.sleep(0.01)