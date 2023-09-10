import socket

severIP = "192.168.0.169"
serverPort = 4010
bufferSize = 1024

serverAddress = (severIP, serverPort);



# message to send back to the client

msgToSend = "Test message from client"

bytesToSend = str.encode(msgToSend)

UDPServerSocket = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)

UDPServerSocket.sendto(bytesToSend, serverAddress)

