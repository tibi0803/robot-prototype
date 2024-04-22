from machine import Pin, PWM, ADC, UART
import utime
from hcsr04 import HCSR04 # for the distance sensor
from time import sleep #   |^
#                                                                  line sensor 

sensor1 = ADC(Pin(27))
sensor1.atten(ADC.ATTN_11DB)
sensor2 = ADC(Pin(26))
sensor2.atten(ADC.ATTN_11DB)
sensor3 = ADC(Pin(25))
sensor3.atten(ADC.ATTN_11DB)
sensor4 = ADC(Pin(33))
sensor4.atten(ADC.ATTN_11DB)
sensor5 = ADC(Pin(32))
sensor5.atten(ADC.ATTN_11DB)

#                                                               motor controller
frequency = 15000

# Pins for Motor 1
pin1_motor1 = Pin(19, Pin.OUT)
pin2_motor1 = Pin(2, Pin.OUT)
enable_motor1 = PWM(Pin(13), frequency)

# Pins for Motor 2
pin1_motor2 = Pin(15, Pin.OUT)
pin2_motor2 = Pin(18, Pin.OUT)
enable_motor2 = PWM(Pin(5), frequency)

#                                                               distance sensor   
sensor = HCSR04(trigger_pin=22, echo_pin=23, echo_timeout_us=10000)

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 5

#define the variable for current state
current_state = 'forward'

#                                                                  MAIN LOOP

while True:
    s1value = sensor1.read()
    s2value = sensor2.read()
    s3value = sensor3.read()
    s4value = sensor4.read()
    s5value = sensor5.read()
    print(s1value,s2value,s3value,s4value,s5value) #for the line sensor

    distance = sensor.distance_cm() #for the distance sensor

    if current_state == 'forward':

        enable_motor1.duty(1023)
        enable_motor2.duty(1023)
        pin1_motor1.value(1)
        pin2_motor1.value(0)
        pin1_motor2.value(1)
        pin2_motor2.value(0)
        # Code for moving forward

        if s1value < 3500 or s2value < 1600:
            current_state = 'turn_left'
            counter = 0
        elif s5value < 3500 or s4value < 1600:
            current_state = 'turn_right'
            counter = 0
        elif distance > 5.0:
            current_state = "stop"
            counter = 0

    
    if current_state == 'turn_right':
        print("turn_right")
        enable_motor1.duty(1023)
        enable_motor2.duty(1023)
        pin1_motor1.value(1)
        pin2_motor1.value(0)
        pin1_motor2.value(0)
        pin2_motor2.value(1)
        # Code for turning right

        # check if it is necessary to update current_state
        if counter == COUNTER_MAX:
            current_state = 'forward'
    
    if current_state == 'turn_left':
        print("turn_left")
        enable_motor1.duty(1023)
        enable_motor2.duty(1023)
        pin1_motor1.value(0)
        pin2_motor1.value(1)
        pin1_motor2.value(1)
        pin2_motor2.value(0)
        # Code for turning left

        # check if it is necessary to update current_state
        if counter == COUNTER_MAX:
            current_state = 'forward'        

    if current_state == "stop": # this has to become turn around and find path
        print("stop")
        enable_motor1.duty(1023)
        enable_motor2.duty(1023)
        pin1_motor1.value(0)
        pin2_motor1.value(0)
        pin1_motor2.value(0)
        pin2_motor2.value(0)
        # Code for stopping

        # check if it is necessary to update current_state
        if counter == COUNTER_MAX:
            current_state = 'forward'

    # increment counter
    counter += 1