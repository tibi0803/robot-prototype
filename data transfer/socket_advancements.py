import socket
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('10.149.34.15', 12345))
s.listen(5)
print('Waiting for connection...')

# Data storage for plotting
x_data = []
y_data = []
buffer = ""

# Lock for thread-safe data access
data_lock = threading.Lock()

def handle_connection(c, addr):
    global buffer
    while True:
        data = c.recv(1024)
        if not data:
            break

        buffer += data.decode('utf-8')

        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            if ',' in line:
                x_value, y_value = line.split(',')
                try:
                    x = float(x_value)
                    y = float(y_value)
                    with data_lock:
                        x_data.append(x)
                        y_data.append(y)
                except ValueError:
                    pass

    c.close()

def start_server():
    while True:
        c, addr = s.accept()
        print('Got connection from', addr)
        t = threading.Thread(target=handle_connection, args=(c, addr))
        t.start()

def update_plot(frame):
    with data_lock:
        plt.cla()
        plt.plot(x_data, y_data, marker='o')
        plt.xlabel('X-coordinate')
        plt.ylabel('Y-coordinate')
        plt.title('Real-time Object Tracking')

fig, ax = plt.subplots()
ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

server_thread = threading.Thread(target=start_server)
server_thread.start()

plt.show()

s.close()
