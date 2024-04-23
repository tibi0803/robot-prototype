import machine
import math
import time

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

# Initialize variables for previous encoder values
prev_left_encoder = left_encoder_pin.value()
prev_right_encoder = right_encoder_pin.value()

def update_odometry():
    # Read current encoder values
    current_left_encoder = left_encoder_pin.value()
    current_right_encoder = right_encoder_pin.value()
    
    # Calculate changes in encoder values
    delta_left_encoder = current_left_encoder - prev_left_encoder
    delta_right_encoder = current_right_encoder - prev_right_encoder
    
    # Calculate linear and angular displacements
    delta_s = R / 2 * (delta_left_encoder + delta_right_encoder)
    delta_theta = R / L * (delta_right_encoder - delta_left_encoder)
    
    # Update position and orientation
    x += delta_s * math.cos(theta)
    y += delta_s * math.sin(theta)
    theta += delta_theta
    
    # Normalize orientation to be within the range [0, 2*pi)
    theta = theta % (2 * math.pi)
    
    # Update previous encoder values
    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder
    
    # Print current position and orientation
    print("Position: ({:.2f}, {:.2f}) cm".format(x, y))
    print("Orientation: {:.2f} radians".format(theta))

