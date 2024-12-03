import socket
server_host = "192.168.50.53"
server_port = 65432

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


client_socket.connect((server_host, server_port))

client_socket.sendall("Hello from the client!".encode())

data = client_socket.recv(1024)

print(f"Received from server : {data.decode()}")

client_socket.close()