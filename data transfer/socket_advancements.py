import socket
import threading

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific IP address and port number
s.bind(('192.168.1.52', 12345))

# Listen for incoming connections
s.listen(5)
print('Waiting for connection...')

while True:
    # Accept an incoming connection
    c, addr = s.accept()
    print('Got connection from', addr)

    # Create a new thread to handle the connection
    t = threading.Thread(target=handle_connection, args=(c, addr))
    t.start()

# Close the socket
s.close()

def handle_connection(c, addr):
    while True:
        # Receive data from the ESP32 board
        data = c.recv(1024)
        if not data:
            break

        # Decode the received data from bytes to string
        data_str = data.decode('utf-8')

        # Split the data into a list
        data_list = data_str.split(',')

        # Print the received data
        print('Received:', data_list)

    # Close the connection
    c.close()