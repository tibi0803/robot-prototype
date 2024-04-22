import machine
import utime

# pin numbers
sensor_pins = [27, 26, 25, 33, 32]  

# Initialise the sensor pins as input
sensors = [machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP) for pin in sensor_pins]

def read_sensors():
    
    return [sensor.value() == 0 for sensor in sensors]  # Assuming reflective surface pulls the pin low

print("Reading IR Line Sensors")

while True:
    sensor_states = read_sensors()
    state_strings = ['Open' if state else 'Covered' for state in sensor_states]
    print("Sensor States:", state_strings)
    
    utime.sleep(0.5)  # Delay for half a second

# STATE machine to show that if middle 3 pins are 'covered' then line is in the middle
# also shows that if all are open turn right to find line
    
## Call the read_sensors() function to get the sensor states
#sensor_states = read_sensors()

## Find the index of the sensor connected to pin 27 (assuming it's the first sensor in the list)
#sensor_27_state = sensor_states[0]

## Print the state of the sensor connected to pin 27
#print(f"Sensor on pin 27 state: {'Open' if sensor_27_state else 'Covered'}")