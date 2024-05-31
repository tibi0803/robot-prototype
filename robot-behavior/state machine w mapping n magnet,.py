from machine import Pin, PWM, ADC, UART, time_pulse_us
import utime
from hcsr04 import HCSR04 # for the distance sensor
from time import sleep #   |^
import math   #encoders
import time
import machine #|^
import socket   #communication w the laptop
import network  #|^
#from collections import deque #mapping (try import *)
import utime # |^

#                                                              network connection

#establish a Wi-Fi connection using the network
#wlan = network.WLAN(network.STA_IF)
#wlan.active(True)
#wlan.connect('House of Mici', '76542527')

#                                                                    magnet

mag = machine.Pin(4, machine.Pin.OUT)
def electromagnet_on():
    mag.on()  # Turn on the electromagnet

def electromagnet_off():
    mag.off()  # Turn off the electromagnet
#                                                                  line sensor 

sensor1 = ADC(Pin(32))  # Valid ADC1 pin
sensor2 = ADC(Pin(33))  # Valid ADC1 pin
sensor3 = ADC(Pin(34))  # Valid ADC1 pin, input-only
sensor4 = ADC(Pin(35))  # Valid ADC1 pin, input-only
sensor5 = ADC(Pin(36))  # Valid ADC1 pin, input-only

# Set the attenuation level (ESP32 ADC valid attenuation levels: 0dB, 2.5dB, 6dB, 11dB)
sensor1.atten(ADC.ATTN_11DB)
sensor2.atten(ADC.ATTN_11DB)
sensor3.atten(ADC.ATTN_11DB)
sensor4.atten(ADC.ATTN_11DB)
sensor5.atten(ADC.ATTN_11DB)

#                                                                color sensor
# Define pins
CS0 = Pin(21, Pin.OUT)
CS1 = Pin(39, Pin.IN)
CS2 = Pin(26, Pin.OUT)
CS3 = Pin(25, Pin.OUT)
CSOUT = Pin(27, Pin.IN)

# Stores frequency read by the photodiodes
redFrequency = 0
greenFrequency = 0
blueFrequency = 0


# Stores the red. green and blue colors
redColor = 0
greenColor = 0
blueColor = 0

# Setting frequency scaling to 20%
CS0.value(1)
CS1.value(0)

# scale values
def scale_value(unscaled, from_min, from_max, to_min, to_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

def getcolor():
    #Setting RED (R) filtered photodiodes to be read
    CS2.value(0)
    CS3.value(0)

    #Reading the output frequency
    redFrequency = time_pulse_us(CSOUT, 0)
    #Remaping the value of the RED (R) frequency from 0 to 255
    redColor = scale_value(redFrequency, 8, 20, 255,0);

    #Setting GREEN filtered photodiodes to be read
    CS2.value(1)
    CS3.value(1)

    #Reading the output frequency
    greenFrequency = time_pulse_us(CSOUT, 0)
    #Remaping the value of the GREEN frequency from 0 to 255
    greenColor = scale_value(greenFrequency, 23, 55, 255,0)

    #Setting BLUE filtered photodiodes to be read
    CS2.value(0)
    CS3.value(1)

    #Reading the output frequency
    blueFrequency = time_pulse_us(CSOUT, 0)
    #Remaping the value of the BLUE frequency from 0 to 255
    blueColor = scale_value(blueFrequency, 11, 43, 255,0)

    if redColor > greenColor and redColor > blueColor:
        currentColor = "red"
    
    if greenColor > redColor and greenColor > blueColor:
        currentColor = "green"
    
    if blueColor > redColor and blueColor > greenColor:
        currentColor = "blue"
    
    print(redColor, greenColor, blueColor, currentColor)
    sleep(0.5)

#                                                        graph that represents the map

graph = {
    'A': {'B': 'left', 'S':'above'},
    'B': {'A': 'right', 'C': 'left' },
    'C': {'B': 'right', 'D': 'left' },
    'D': {'C': 'right', 'E': 'left' },
    'E': {'F': 'left', 'H': 'above', 'D': 'right'},
    'F': {'G': 'above', 'E': 'right'},
    'G': {'J': 'above', 'F': 'below', 'H': 'right'},
    'H': {'G': 'left', 'I': 'above', 'E': 'below'},
    'I': {'J': 'left', 'H': 'below', 'Q': 'above', 'S': 'right'},
    'J': {'I': 'right', 'G': 'below', 'K': 'above'},
    'K': {'J': 'below', 'L': 'right'},
    'L': {'K': 'left', 'M': 'right'},
    'M': {'L': 'left', 'N': 'right'},
    'N': {'M': 'left', 'O': 'right'},
    'O': {'N': 'left', 'Q': 'below', 'P': 'right'},
    'P': {'O': 'left', 'R': 'below'},
    'Q': {'I': 'below', 'O': 'above', 'R': 'right'},
    'R': {'Q': 'left', 'P': 'above', 'S': 'below'},
    'S': {'R': 'right', 'I': 'below', 'A': 'below'}
}

#                                                              BFS for pathfinding
#start = 'E'
#start = graph['E']
#goal = graph['M']
#this function finds the path

def bfs(graph, start, goal):
    queue = [(start, [start])]
    visited = set()
    
    while queue:
        vertex, path = queue.pop(0)  # pop from the front of the list
        if vertex in visited:
            continue
        
        visited.add(vertex)
        
        for neighbor, direction in graph[vertex].items():
            if neighbor == goal:
                return path + [neighbor]
            
            queue.append((neighbor, path + [neighbor]))
    return None

#                                                             intersection detection
def detect_intersection():
    if s1value < 3500 or s5value < 3500:
        return True
    

#                                                               Navigate the path
# Possible orientations
orientations = ['north', 'east', 'south', 'west']

# Initial orientation
current_orientation = 'north'

# Function to update orientation based on turn
def update_orientation(turn):
    global current_orientation
    idx = orientations.index(current_orientation)
    if turn == 'left':
        current_orientation = orientations[(idx + 1) % 4]
    elif turn == 'right':
        current_orientation = orientations[(idx - 1) % 4]
    elif turn == 'turnaround':
        current_orientation = orientations[(idx + 2) % 4]

# Function to get direction relative to current orientation
def get_relative_direction(direction):
    idx = orientations.index(current_orientation)
    if direction == 'above':
        return orientations[idx]
    elif direction == 'right':
        return orientations[(idx + 1) % 4]
    elif direction == 'below':
        return orientations[(idx + 2) % 4]
    elif direction == 'left':
        return orientations[(idx + 3) % 4]

# Navigate the path with dynamic orientation
def navigate_path(path):
    global current_orientation
    current_position = path[0]
    for next_position in path[1:]:
        direction = graph[current_position][next_position]
        relative_direction = get_relative_direction(direction)
        
        print(f"Move {relative_direction} to {next_position}")

        if relative_direction == 'north':
            print('going straight')
            while not detect_intersection():
                follow_line()
            current_position = next_position
            utime.sleep(1)  # Pause briefly at each intersection

        elif relative_direction == 'west':
            leftturn()
            print('turning left')
            update_orientation('left')
            while not detect_intersection():
                follow_line()
            current_position = next_position
            utime.sleep(1)  # Pause briefly at each intersection

        elif relative_direction == 'east':
            rightturn()
            print('turning right')
            update_orientation('right')
            while not detect_intersection():
                follow_line()
            current_position = next_position
            utime.sleep(1)  # Pause briefly at each intersection

        elif relative_direction == 'south':
            turnaround()
            print('turning around')
            update_orientation('turnaround')
            while not detect_intersection():
                follow_line()
            current_position = next_position
            utime.sleep(1)  # Pause briefly at each intersection
        
        #while not detect_intersection():
        #    follow_line()
        #   print('just following the line')

        #current_position = next_position
        #utime.sleep(1)  # Pause briefly at each intersection

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

#                                         counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 5

#define the variable for                                         current state
#current_state = 'forward'

#                                                                  encoders
# Define GPIO pins for wheel encoders
left_encoder_pin = machine.Pin(12, machine.Pin.IN)
right_encoder_pin = machine.Pin(14, machine.Pin.IN)

# Define constants for wheel parameters
WHEEL_RADIUS = 33.5  # Radius of the wheels in mm
WHEEL_DISTANCE = 200  # Distance between the wheels in mm

#distance traveled by a wheel every encoder pulse
dist = 5  #in mm

# Initialize variables for robot's position and orientation
x = 0  # Initial x-coordinate (in mm)
y = 0  # Initial y-coordinate (in mm)
theta = 0  # Initial orientation (in radians)

#encoder counts
left_encoder_count = 0
right_encoder_count = 0

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
        
        prev_left_encoder_state = 1
        
        # Send x and y coordinates as a comma-separated string
        data = f"{x},{y}\n"
        s.send(data.encode())

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
        
        prev_right_encoder_state = 1
        
        # Send x and y coordinates as a comma-separated string
        data = f"{x},{y}\n"
        s.send(data.encode())


    # Update previous encoder states 
    if right_state == 0:
        
        prev_right_encoder_state = right_state

    if left_state == 0:
        
        prev_left_encoder_state = left_state

    # Normalize robot orientation to the range [-pi, pi]
    theta = theta % (2 * math.pi)

def follow_line():
    if s3value < 3500:
        current_state = 'forward'
        counter = 0
    elif s2value < 1600:
        current_state = 'turn_left'
        counter = 0
    elif s4value < 1600:
        current_state = 'turn_right'
        counter = 0

    if current_state == 'forward': #state in which there is FOLLOW THE LINE
        #print("forward")
        forward() #robot moves forward
        # Call the update_odometry function to update the current position and orientation
        update_odometry()

        # return to going forward
        if counter == COUNTER_MAX:
            current_state = 'forward'

    if current_state == 'turn_right':
        #print("turn_right")
        turnright()
        # Call the update_odometry function to update the current position and orientation
        update_odometry()

        # return to going forward
        if counter == COUNTER_MAX:
            current_state = 'forward'
    
    if current_state == 'turn_left':
        #print("turn_left")
        turnleft()

        # Call the update_odometry function to update the current position and orientation
        update_odometry()

        # return to going forward
        if counter == COUNTER_MAX:
            current_state = 'forward'
    
def forward():
    enable_motor1.duty(1023)
    enable_motor2.duty(1023)
    pin1_motor1.value(1)
    pin2_motor1.value(0)
    pin1_motor2.value(1)
    pin2_motor2.value(0)
    # Code for moving forward

def turnleft():
    enable_motor1.duty(1023)
    enable_motor2.duty(1023)
    pin1_motor1.value(0)
    pin2_motor1.value(0)
    pin1_motor2.value(1)
    pin2_motor2.value(0)
    # Code for turning left

def turnright():
    enable_motor1.duty(1023)
    enable_motor2.duty(1023)
    pin1_motor1.value(1)
    pin2_motor1.value(0)
    pin1_motor2.value(0)
    pin2_motor2.value(0)
    # Code for turning right

def stop(): # make the robot stop
    enable_motor1.duty(1023)
    enable_motor2.duty(1023)
    pin1_motor1.value(0)
    pin2_motor1.value(0)
    pin1_motor2.value(0)
    pin2_motor2.value(0)
    # Code for stopping

# tunrs for navigation need to be tested in action and calibrated so that the robot lands in the middle of the line in the direction it turned
def leftturn():
    enable_motor1.duty(1023)
    enable_motor2.duty(1023)
    pin1_motor1.value(0)
    pin2_motor1.value(0)
    pin1_motor2.value(1)
    pin2_motor2.value(0)
    # Code for turning left

def rightturn():
    enable_motor1.duty(1023)
    enable_motor2.duty(1023)
    pin1_motor1.value(1)
    pin2_motor1.value(0)
    pin1_motor2.value(0)
    pin2_motor2.value(0)
    # Code for turning right

def turnaround():
    enable_motor1.duty(1023)
    enable_motor2.duty(1023)
    pin1_motor1.value(1)
    pin2_motor1.value(0)
    pin1_motor2.value(0)
    pin2_motor2.value(1)
    # Code for turning aroumd




#                                                                  MAIN LOOP
#while not wlan.isconnected():
    #pass
#print('Connected to Wi-Fi')


#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#connect to the laptop using the socket object
#s.connect(('192.168.0.101', 12345)) #replace the values

#you can send data to the laptop using the send() method
#s.send('Hello, laptop!')

#   ^ part for connecting to the laptop

while True:
    s1value = sensor1.read()
    s2value = sensor2.read()
    s3value = sensor3.read()
    s4value = sensor4.read()
    s5value = sensor5.read()

    path = bfs(graph, 'E', 'M')

    if path:
        print("Path found:", path)
    else:
        print("No path found")
    
    navigate_path(path)    

    # increment counter
    counter += 1
