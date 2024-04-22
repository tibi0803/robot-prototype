from machine import Pin
import time

# Define pin for the encoder
encoder_pin = Pin(12, Pin.IN)

# Variable to count encoder steps
encoder_count = 0

# Main loop
while True:
    # Read current state of pin
    current_state = encoder_pin.value()
    
    # Check for rising edge (transition from LOW to HIGH)
    if current_state == 1:
        # Increment encoder count
        encoder_count += 1
        
        # Print current encoder count
        print("Encoder count:", encoder_count)
    
    # Small delay to debounce
    time.sleep_ms(10)


#wheels have the radius of 33.5mm 
#circumference is 210.5 mm
#each pulse is 10.5 mm