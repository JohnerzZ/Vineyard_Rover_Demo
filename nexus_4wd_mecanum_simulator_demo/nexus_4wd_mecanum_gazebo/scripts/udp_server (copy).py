import socket

localIP = "192.168.0.169"
localPort = 51356
bufferSize = 1024

# message to send back to the client

msgFromServer = "Hello UDP Client\n"

bytesToSend = str.encode(msgFromServer)

UDPServerSocket = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)

UDPServerSocket.bind((localIP, localPort))

print("UDP server up and listening")

#listen for incoming datagrams
while(True):
	bytesAddressPair = UDPServerSocket.recvfrom(bufferSize) #receive both the data and the address from which they come from
	message = bytesAddressPair[0]
	address = bytesAddressPair[1]

	clientMsg = "Message from Client: " + message.decode()
	clientIP = "Client IP Address: " + address

	print(clientMsg)
	print(clientIP)

	UDPServerSocket.sendto(bytesToSend, address)

