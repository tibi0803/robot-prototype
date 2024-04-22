import machine
import time

# Define the GPIO pin number for the output pin of the encoder
encoder_pin_out = 14  # Pin OUT of the encoder

# Initialize the output pin as an input
pin_out = machine.Pin(encoder_pin_out, machine.Pin.IN)

# Main loop to read and print encoder state
while True:
    # Read the state of the output pin
    state_out = pin_out.value()
    
    # Print the state of the output pin
    print("Output pin state:", state_out)
    
    # Add a delay to control the loop frequency
    time.sleep(1)  # Adjust as needed
