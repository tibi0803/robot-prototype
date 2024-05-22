import socket
import threading

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific IP address and port number
s.bind(('192.168.0.100', 12345))

# Listen for incoming connections
s.listen(5)
print('Waiting for connection...')

def handle_connection(c, addr):
    buffer = ""
    while True:
        # Receive data from the ESP32 board
        data = c.recv(1024)
        if not data:
            break

        # Decode the received data from bytes to string
        buffer += data.decode('utf-8')

        # Process complete pairs of x, y values
        while ',' in buffer:
            x_index = buffer.find(',')
            if x_index == -1:
                break

            # Split the data up to the first comma
            x_value = buffer[:x_index]
            buffer = buffer[x_index + 1:]

            # Find the next comma for the y value
            y_index = buffer.find(',')
            if y_index == -1:
                # If there is no comma, wait for more data
                break

            # Get the y value
            y_value = buffer[:y_index]
            buffer = buffer[y_index + 1:]

            # Print the received data pair
            print('Received:', (x_value, y_value))

    # Close the connection
    c.close()

while True:
    # Accept an incoming connection
    c, addr = s.accept()
    print('Got connection from', addr)

    # Create a new thread to handle the connection
    t = threading.Thread(target=handle_connection, args=(c, addr))
    t.start()

# Close the socket
s.close()
