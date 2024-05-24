from machine import Pin, PWM, ADC, UART
import utime
#from hcsr04 import HCSR04 # for the distance sensor
from time import sleep #   |^
import math   #encoders
import time
import machine #|^
import socket   #communication w the laptop
import network  #|^
from collections import deque #mapping
import utime# |^
frequency = 15000

# Pins for Motor 1
pin1_motor1 = Pin(19, Pin.OUT)
pin2_motor1 = Pin(2, Pin.OUT)
enable_motor1 = PWM(Pin(13), frequency)

# Pins for Motor 2
pin1_motor2 = Pin(15, Pin.OUT)
pin2_motor2 = Pin(18, Pin.OUT)
enable_motor2 = PWM(Pin(5), frequency)

while True:
    enable_motor1.duty(1023)
    enable_motor2.duty(1023)
    pin1_motor1.value(1)
    pin2_motor1.value(0)
    pin1_motor2.value(1)
    pin2_motor2.value(0)
