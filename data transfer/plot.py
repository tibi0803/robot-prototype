import matplotlib.pyplot as plt

plt.ion()

def plot_data(data_list):
    data = [float(x) for x in data_list]
    plt.clf()
    plt.plot(data)
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('ESP32 Data')
    plt.pause(0.01)

while True:
    c, addr = s.accept()
    print('Got connection from', addr)
    while True:
        data = c.recv(1024)
        if not data:
            break
        print('Received:', data)
        data_list = data.decode('utf-8').split(',')
        plot_data(data_list)
    c.close() /