import socket
import struct
import numpy as np

def send_data(socket, data):
    socket.sendall(struct.pack('>I', len(data)) + data)

def recv_data(socket):
    data_len = socket.recv(4)
    if not data_len: return None

    data_len = struct.unpack('>I', data_len)[0]
    return socket.recv(data_len)

host_ip = "0.0.0.0"
host_port = 4444

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as socket:
    socket.bind((host_ip, host_port))
    socket.listen()

    print('Waiting for connection...')
    conn, addr = socket.accept()

    with conn:
        print("Connected")

        data = recv_data(conn)
        if data != None:
            data = data.decode('utf-8')
        print(data)

        send_data(conn, b'Hello back')
