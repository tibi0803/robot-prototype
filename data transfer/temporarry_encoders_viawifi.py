from machine import Pin, PWM, ADC, UART
import utime
import math   #encoders
import time
import machine #|^

import socket
import network

#establish a Wi-Fi connection using the network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect('TELE2-EF3615_2.4G', '477463EFE7CA')

#After attempting to connect to the Wi-Fi network, you need to wait for the connection to be established.
while not wlan.isconnected():
    pass
print('Connected to Wi-Fi')

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect(('192.168.1.52', 12345)) #replace the values

#you can send data to the laptop using the send() method
s.send('Hello, laptop!')

#                                                                  encoders
# Define GPIO pins for wheel encoders
left_encoder_pin = machine.Pin(12, machine.Pin.IN)
right_encoder_pin = machine.Pin(13, machine.Pin.IN)

# Define constants for wheel parameters
WHEEL_RADIUS = 33.5  # Radius of the wheels in mm
WHEEL_DISTANCE = 200  # Distance between the wheels in mm

#distance traveled by a wheel every encoder pulse
dist = 5  #in mm

# Initialize variables for robot's position and orientation
x = 0  # Initial x-coordinate (in mm)
y = 0  # Initial y-coordinate (in mm)
theta = 0  # Initial orientation (in radians)

# Variables to keep track in the change of movement of the wheels
delta_left = 0
delta_right = 0

# Variables to store previous encoder counts
prev_left_encoder_count = 0
prev_right_encoder_count = 0

#Variables to store previous encoder states
prev_left_encoder_state = 0
prev_right_encoder_state = 0

def update_odometry():
    global x, y, theta, prev_left_encoder_count, prev_right_encoder_count, left_encoder_count, right_encoder_count, prev_left_encoder_state, prev_right_encoder_state, delta_right, delta_left, Rturn

    # Read current states of encoder pins
    left_state = left_encoder_pin.value()
    right_state = right_encoder_pin.value()

    # Check for rising edge (transition from LOW to HIGH) for the left wheel
    if left_state == 1 and prev_left_encoder_state == 0:
        left_encoder_count += 1

        # Calculate the change in distance for a pulse for the left wheel
        delta_left = (left_encoder_count - prev_left_encoder_count) * dist
        
        #update the orientation of the robot
        theta += (delta_left - delta_right)/WHEEL_DISTANCE

        #radius of the turn   -  useful for other formulas
        Rturn = (delta_left + delta_right)/2*theta

        if delta_left - delta_right >= 0:
            #formulas highlited in blue in notes
            x += Rturn*math.sin(theta)
            y += Rturn*(1-math.cos(theta))

        elif delta_left - delta_right < 0:
            x += (delta_left - delta_right)/2
            y += (delta_left**2 - delta_right**2)/4*WHEEL_DISTANCE

    # Check for rising edge (transition from LOW to HIGH) for the right wheel
    if right_state == 1 and prev_right_encoder_state == 0:
        right_encoder_count += 1

        # Calculate the change in distance for a pulse for the left wheel
        delta_left = (left_encoder_count - prev_left_encoder_count) * dist
        
        #update the orientation of the robot
        theta += (delta_left - delta_right)/WHEEL_DISTANCE

        #radius of the turn   -  useful for other formulas
        Rturn = (delta_left + delta_right)/2*theta

        if delta_left - delta_right >= 0:
            #formulas highlited in blue in notes
            x += Rturn*math.sin(theta)
            y += Rturn*(1-math.cos(theta))

        elif delta_left - delta_right < 0:
            x += (delta_left - delta_right)/2
            y += (delta_left**2 - delta_right**2)/4*WHEEL_DISTANCE

    # Update previous encoder states 
    if right_state == 0:
        
        prev_right_encoder_state = right_state

    if left_state == 0:
        
        prev_left_encoder_state = left_state

    # Normalize robot orientation to the range [-pi, pi]
    theta = theta % (2 * math.pi)

#----------------------------------------------------

while wlan.isconnected():

    update_odometry()
    s.send(str(x) + ',' + str(y))
    