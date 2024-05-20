import socket
import threading

#In this example, the socket is bound to the IP address '192.168.1.2'
#and port number '12345'. You should replace these values 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('192.168.1.52', 12345))

#After binding the socket, you need to listen for incoming connections. 
s.listen(5)
print('Waiting for connection...')

#Once a connection is established, you can receive data from the esp32 
#board using the recv() method.

while True:
    c, addr = s.accept()
    print('Got connection from', addr)
    while True:
        data = c.recv(1024)
        if not data:
            break
        print('Received:', data)
    c.close()

#In this example, the accept() method is used to accept an incoming 
#connection. The recv() method is then used to receive data from the esp32 board. 
#The while True loop ensures that the server continues to listen for incoming connections.