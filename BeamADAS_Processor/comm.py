import socket
import struct
import time

class Comm:
    def __init__(self, proc):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.proc = proc
        if proc == 0:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(("0.0.0.0", 4441))
            self.socket.listen()

            print('Waiting for connection...')
            self.conn, self.addr = self.socket.accept()
            print("Connected")
        else:
            time.sleep(3)

            self.socket.connect(("!!!host_ip!!!", 4441 + proc))

    def send_data(self, type, data):
        if type == b'I':
            self.socket.sendall(struct.pack('>Bff', type, data[0], data[1]))
        elif type == b'B':
            self.socket.sendall(struct.pack('>BII', type, data[0], data[1]))

    def recv_data(self):
        recv = self.conn.recv(1)
        if len(recv) < 1: return None, None, None, None, None, None

        data_len = 0
        timestamp = 0.0
        veh_dir = [0.0, 0.0]
        gear = 'N'

        data_type = struct.unpack('>B', recv)

        if data_type == 'L':
            recv = self.conn.recv(16)
            if len(recv) < 16: return None, None, None, None, None, None

            data_len, timestamp, veh_dir[0], veh_dir[1] = struct.unpack('>Ifff', recv)
        elif data_type == 'P':
            recv = self.conn.recv(5)
            if len(recv) < 5: return None, None, None, None, None, None

            timestamp, gear = struct.unpack('>fB', recv)
            data_len = 24
        elif data_type == 'C':
            recv = self.conn.recv(4)
            if len(recv) < 4: return None, None, None, None, None, None

            data_len = 921600
            timestamp = struct.unpack('>f', recv)
        else:
            recv = self.conn.recv(4)
            if len(recv) < 4: return None, None, None, None, None, None

            data_len = struct.unpack('>I', recv)

        data = b''
        read_len = 0
        while data_len > read_len:
            data += self.conn.recv(data_len - read_len)
            read_len = len(data)

        return data_type, data_len, timestamp, veh_dir, gear, data

    def close(self):
        self.conn.close()
        self.socket.close()
