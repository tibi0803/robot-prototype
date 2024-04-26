import machine
import time

# Define the GPIO pin number
pin_number = 12  # Change this to the GPIO pin number you are using

# Initialize the pin as an input
pin = machine.Pin(pin_number, machine.Pin.IN)

# Initialize count variable
count = 0

# Initialize previous pin state
prev_pin_state = 0

# Main loop to count pin state changes
while True:
    # Read the state of the pin
    pin_state = pin.value()
    
    # Check if the pin state has changed from low to high
    if pin_state == 1 and prev_pin_state == 0:
        # Increment count
        count += 1
        
        # Print the current count
        print("Pin high count:", count)
    
    # Update previous pin state
    prev_pin_state = pin_state
    
    # Add a delay to control the loop frequency
    time.sleep(0.1)  # Adjust as needed

