import socket
import struct

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
            self.socket.connect(("!!!host_ip!!!", 4441 + proc))

    def send_data(self, type, data):
        self.socket.sendall(struct.pack('>BI', type, len(data)) + data)

    def recv_data(self):
        recv = self.conn.recv(5)
        if len(recv) < 5: return None, None, None, None, None

        data_type, data_len = struct.unpack('>BI', recv)
        read_len = 0
        data = b''

        veh_dir = None
        timestamp = None

        if data_type == 'L':
            recv = self.conn.recv(8)
            if len(recv) == 8:
                veh_dir = [0.0, 0.0]
                timestamp, veh_dir[0], veh_dir[1] = struct.unpack('>fff', recv)

        while data_len > read_len:
            data += self.conn.recv(data_len - read_len)
            read_len = len(data)
            
        return data_type, data_len, timestamp, veh_dir, data

    def close(self):
        self.conn.close()
        self.socket.close()        
