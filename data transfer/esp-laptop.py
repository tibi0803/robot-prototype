#mport the necessary libraries to establish a Wi-Fi connection and send data to the laptop
#The socket library is used to create a socket object, which is necessary for sending
#data over the network. The network library is used to establish a Wi-Fi connection.
import socket
import network

#establish a Wi-Fi connection using the network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect('TELE2-EF3615_2.4G', '477463EFE7CA')

#After attempting to connect to the Wi-Fi network, you need to wait for the connection to be established.
while not wlan.isconnected():
    pass
print('Connected to Wi-Fi')

#you can create a socket object using the socket. This creates a socket object that can be 
#used to send and receive data over the network.
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#connect to the laptop using the socket object
s.connect(('192.168.1.52', 12345)) #replace the values

#you can send data to the laptop using the send() method
s.send('Hello, laptop!')

#Finally, you need to close the connection using the close() method.
s.close()