import socket
import struct

class Comm:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("0.0.0.0", 4444))
        self.socket.listen()

        print('Waiting for connection...')
        self.conn, self.addr = self.socket.accept()
        print("Connected")

    def send_data(self, data):
        self.conn.sendall(struct.pack('>I', len(data)) + data)

    def recv_data(self):
        data_len = self.conn.recv(4)
        if not data_len: return None

        data_len = struct.unpack('>I', data_len)[0]
        read_len = 0
        data = b''

        while data_len > read_len:
            data += self.conn.recv(data_len - read_len)
            read_len = len(data)
            
        return data

    def close(self):
        self.conn.close()
        self.socket.close()        
