from machine import Pin, PWM, ADC, UART
import utime
from hcsr04 import HCSR04 # for the distance sensor
from time import sleep #   |^
import math   #encoders
import time
import machine #|^
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

#                                                             encoders
# Define GPIO pins for wheel encoders
left_encoder_pin = machine.Pin(12, machine.Pin.IN)
right_encoder_pin = machine.Pin(13, machine.Pin.IN)

# Define constants for wheel parameters
R = 3.35  # Radius of the wheels (in cm)
L = 20  # Distance between the wheels (in cm)

# Initialize variables for robot's position and orientation
x = 0  # Initial x-coordinate (in cm)
y = 0  # Initial y-coordinate (in cm)
theta = 0  # Initial orientation (in radians)

# Variables to count encoder steps for each wheel
left_encoder_count = 0
right_encoder_count = 0

# Variables to store previous encoder counts
prev_left_encoder_count = 0
prev_right_encoder_count = 0

def update_odometry():
    global x, y, theta, prev_left_encoder_count, prev_right_encoder_count, left_encoder_count, right_encoder_count, prev_left_encoder_state, prev_right_encoder_state

    # Read current states of encoder pins
    left_state = left_encoder_pin.value()
    right_state = right_encoder_pin.value()

    # Check for rising edge (transition from LOW to HIGH) for the left wheel
    if left_state == 1 and prev_left_encoder_state == 0:
        left_encoder_count += 1

        print('x value:', x, 'y value:', y, 'theta value', theta)

    # Check for rising edge (transition from LOW to HIGH) for the right wheel
    if right_state == 1 and prev_right_encoder_state == 0:
        right_encoder_count += 1

        print('x value:', x, 'y value:', y, 'theta value', theta)

    # Update previous encoder states
    prev_left_encoder_state = left_state
    prev_right_encoder_state = right_state

    # Calculate the change in encoder counts for each wheel
    delta_left_encoder = left_encoder_count - prev_left_encoder_count
    delta_right_encoder = right_encoder_count - prev_right_encoder_count

    # Update previous encoder counts
    prev_left_encoder_count = left_encoder_count
    prev_right_encoder_count = right_encoder_count

    # Calculate linear and angular distances traveled by each wheel
    left_distance = 2 * math.pi * WHEEL_RADIUS * left_encoder_count
    right_distance = 2 * math.pi * WHEEL_RADIUS * right_encoder_count

    # Calculate linear and angular distances traveled by the robot
    linear_distance = (left_distance + right_distance) / 2
    angular_distance = (right_distance - left_distance) / WHEEL_DISTANCE

    # Update robot pose using odometry equations
    x += linear_distance * math.cos(theta + angular_distance / 2)
    y += linear_distance * math.sin(theta + angular_distance / 2)
    theta += angular_distance

    # Normalize robot orientation to the range [-pi, pi]
    theta = theta % (2 * math.pi)


#                                                                  MAIN LOOP

while True:
    s1value = sensor1.read()
    s2value = sensor2.read()
    s3value = sensor3.read()
    s4value = sensor4.read()
    s5value = sensor5.read()
    print(s1value,s2value,s3value,s4value,s5value) #for the line sensor

    distance = sensor.distance_cm() #for the distance sensor
    
    # Call the update_odometry function to update the current position and orientation
    update_odometry()

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