from machine import Pin
import time
import math

# Constants
WHEEL_RADIUS = 33.5  # Radius of the wheels in meters
WHEEL_DISTANCE = 200  # Distance between the wheels in meters

# Define pins for the encoders
left_encoder_pin = Pin(12, Pin.IN)
right_encoder_pin = Pin(13, Pin.IN)

# Variables to count encoder steps for each wheel
left_encoder_count = 0
right_encoder_count = 0

# Variables to store previous encoder counts
prev_left_encoder_count = 0
prev_right_encoder_count = 0

# Initialize variables for robot's position and orientation
x = 0  # Initial x-coordinate (in cm)
y = 0  # Initial y-coordinate (in cm)
theta = 0  # Initial orientation (in radians)



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

while True:
    # Other state machine code here

    # Update odometry
    update_odometry()
    # Other state machine code here
    # print('x value:', x, 'y value:', y, 'theta value', theta)

    # Add a delay to control the loop frequency
    #time.sleep(0.1)  # no need for a debounce
