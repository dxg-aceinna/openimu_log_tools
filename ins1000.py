import math
import serial
import serial.tools.list_ports
import struct
import numpy as np

nav_size = 127
nav_header = bytearray.fromhex('af 20 05 0d')

class ins1000:
    def __init__(self, port, baud=230400, pipe=None):
        '''Initialize and then start ports search and autobaud process
        '''
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port, self.baud)
        self.open = self.ser.isOpen()
        self.latest = []
        self.pipe = pipe

    def start(self):
        if self.open:
            bf = bytearray(nav_size*2)
            n_bytes = 0
            while True:
                data = self.ser.read(nav_size)
                ## parse new
                n = len(data)
                for i in range(n):
                    bf[n_bytes + i] = data[i]
                n_bytes += n
                if n_bytes >= nav_size:
                    if bf[0] == nav_header[0] and bf[1] == nav_header[1] and\
                       bf[2] == nav_header[2] and bf[3] == nav_header[3]:
                        self.latest = parse_nav(bf[6:nav_size])
                        if self.pipe is not None:
                                self.pipe.send(self.latest)
                        for i in range(n_bytes-nav_size):
                            bf[i] = bf[i+nav_size]
                        n_bytes = n_bytes - nav_size
                    else:
                        n_bytes = sync_packet(bf, n_bytes, nav_header)
                    # print(''.join('{:#x} '.format(x) for x in bf) )
    def get_latest(self):
        return self.latest

def parse_nav(payload):
    time_idx0 = 0
    lat_idx0 = 8
    lon_idx0 = 16
    alt_idx0 = 24
    vel_idx0 = 28
    quat_idx0 = 40
    time = struct.unpack('d', payload[time_idx0:time_idx0+8])
    lla = np.array([struct.unpack('d', payload[lat_idx0:lat_idx0+8]),\
                    struct.unpack('d', payload[lon_idx0:lon_idx0+8]),\
                    struct.unpack('f', payload[alt_idx0:alt_idx0+4])])
    vel = np.array(struct.unpack('fff', payload[vel_idx0:vel_idx0+12]))
    quat = np.array(struct.unpack('ffff', payload[quat_idx0:quat_idx0+16]))

    return time, lla, vel, quat

def sync_packet(bf, bf_len, header):
    '''
    sync packet when the header of the packet is not at the beginning of the buffer
    '''
    idx = -1
    # print(''.join('{:#2x} '.format(x) for x in bf) )
    while bf_len >= nav_size:
        idx = bf.find(0xaf, idx+1, bf_len) 
        if idx > 0:
            for i in range(1, min(bf_len-idx, len(header))):
                if bf[idx+i] != header[i]:
                    continue
            bf_len = bf_len - idx
            for i in range(bf_len):
                bf[i] = bf[idx+i]
        else:
            bf_len = 0
    # print(''.join('{:#2x} '.format(x) for x in bf) )
    return bf_len

def calc_crc(payload):
    crc = 0x1D0F
    for bytedata in payload:
        crc = crc^(bytedata << 8) 
        for i in range(0,8):
            if crc & 0x8000:
                crc = (crc << 1)^0x1021
            else:
                crc = crc << 1

    crc = crc & 0xffff
    return crc


if __name__ == "__main__":
    ref = ins1000('COM19')
    ref.start()