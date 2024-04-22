from machine import Pin, ADC, UART, time_pulse_us
from time import sleep

# Define pins
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

# scale values
def scale_value(unscaled, from_min, from_max, to_min, to_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

while True:
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