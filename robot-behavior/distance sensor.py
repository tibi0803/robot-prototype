from hcsr04 import HCSR04
from time import sleep

# ESP32
sensor = HCSR04(trigger_pin=22, echo_pin=23, echo_timeout_us=10000)



while True:
    distance = sensor.distance_cm()    
    if distance > 5.0:
        print('Distance:', distance, 'cm')
        print("NO object") # would be replaced by code that tells the robot to continue following line
    if distance < 5.0:
        print('Distance:', distance, 'cm')
        print("Object, go around") #would be replaced with code that makes the robot turn left or right to avoid obstacles
sleep(1)                                                                                                   