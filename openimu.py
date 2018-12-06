import math
import serial
import serial.tools.list_ports
import struct

z1_size = 47
z1_header = bytearray.fromhex('5555')
class openimu:
    def __init__(self, port, baud=115200, pipe=None):
        '''Initialize and then start ports search and autobaud process
        '''
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port, self.baud)
        self.open = self.ser.isOpen()
        self.latest = []
        self.ready = False
        self.pipe = pipe

    def start(self):
        if self.open:
            bf = bytearray(z1_size*2)
            n_bytes = 0
            while True:
                data = self.ser.read(z1_size)
                ## parse new
                n = len(data)
                for i in range(n):
                    bf[n_bytes + i] = data[i]
                n_bytes += n
                while n_bytes >= z1_size:
                    if bf[0] == 0x55 and bf[1] == 0x55:
                        # crc
                        packet_crc = 256 * bf[z1_size-2] + bf[z1_size-1]
                        calculated_crc = calc_crc(bf[2:bf[4]+5])
                        # decode
                        if packet_crc == calculated_crc:
                            self.latest = parse_z1(bf[5:bf[4]+5])
                            # print(self.latest)
                            if self.pipe is not None:
                                self.pipe.send(self.latest)
                            # remove decoded data from the buffer
                            n_bytes -= z1_size
                            for i in range(n_bytes):
                                bf[i] = bf[i+z1_size]
                        else:
                            print('crc fail')
                            n_bytes = sync_packet(bf, n_bytes, z1_header)
                    else:
                        n_bytes = sync_packet(bf, n_bytes, z1_header)

    def get_latest(self):
        return self.latest

def parse_z1(payload):
    '''
    parse z1 packet
    '''
    fmt = 'Ifffffffff'
    data = struct.unpack(fmt, payload)
    timer = data[0]
    acc = data[1:4]
    gyro = data[4:7]
    return timer, acc, gyro

def sync_packet(bf, bf_len, header):
    idx = bf.find(0x55, 0, bf_len) 
    if idx > 0 and bf[idx+1] == 0x55:
        bf_len = bf_len - idx
        for i in range(bf_len):
            bf[i] = bf[i+idx]
    else:
        bf_len = 0
    return bf_len

def calc_crc(payload):
    '''Calculates CRC per 380 manual
    '''
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