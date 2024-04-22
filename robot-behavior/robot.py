import time
from time import sleep
import machine
from machine import Pin, Timer, ADC, UART, time_pulse_us
import utime
from hcsr04 import HCSR04

#--------------------------
# Initialize pins for COLOR SENSOR

CS0 = Pin(32, Pin.OUT)
CS1 = Pin(35, Pin.IN)
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

#--------------------------

# Define pin for the ENCODER
encoder_pin = Pin(12, Pin.IN)

# Variable to count encoder steps
encoder_count = 0
#define the distance with a ratio formula
#--------------------------

# pin numbers for LINE SENSOR
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

#--------------------------

#DISTANCE SENSOR

sensor = HCSR04(trigger_pin=22, echo_pin=23, echo_timeout_us=10000)

#--------------------------

#                              FUNCTIONS

#--------------------------

#color sensor
# scale values
def scale_value(unscaled, from_min, from_max, to_min, to_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

def seecolor(self):

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
        #Return currentcolor??? to use in a state instead of having a readcolor state by itself


#--------------------------

#distance sensor
def getdistance():
    distance = sensor.distance_cm()
    if distance < 5.0:
        #turnaround

#--------------------------
        
#                            STATE MACHINE

class statemachine:

    def __init__(self):
        # Define states and initial state
        self.states = {
            'forward': self.forward,
            'turn_left': self.turn_left,
            'turn_right':self.turn_right,
        }


    def forward(self):
        print("forward")
        #put the code for running the motors at the same speed 
        
    def turn_left(self):
        print("turn_left")
        #put the code for running the left motor at a lower speed

    #def turn_sharpleft(self): #maybe unnecessary 
        #put the code for running the left motor even slower    
    
    def turn_right(self):
        print("turn_right")
        
    #def turn_sharpright(self):
        
    

#                            MAIN LOOP

while True:
    self.states[self.current_state]() #run current state 

    #line sensor reading
    s1value = sensor1.read()
    s2value = sensor2.read()
    s3value = sensor3.read()
    s4value = sensor4.read()
    s5value = sensor5.read()

     

    if s3value < 2000:
        self.current_state = 'forward' 
        #keep going as lower values mean black
           
    elif s1value < 3500 or s2value < 1600:
        self.current_state = 'turn_left'
    
    elif s5value < 3500 or s4value < 1600:
        self.current_state = 'turn_right'
