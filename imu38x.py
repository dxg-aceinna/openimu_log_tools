import math
import serial
import serial.tools.list_ports
import struct

packet_header = bytearray.fromhex('5555')
# payload + 2-byte header + 2-byte type + 1-byte len + 2-byte crc
packet_def = {'A1': [39, bytearray.fromhex('4131')],\
              'A2': [37, bytearray.fromhex('4132')],\
              'S0': [37, bytearray.fromhex('5330')],\
              'S1': [31, bytearray.fromhex('5331')],\
              'z1': [47, bytearray.fromhex('7a31')],\
              'a2': [55, bytearray.fromhex('6132')],\
              'e1': [82, bytearray.fromhex('6531')],\
              'e2': [130, bytearray.fromhex('6532')],\
              'id': [154, bytearray.fromhex('6964')]}

class imu38x:
    def __init__(self, port, baud=115200, packet_type='A2', pipe=None):
        '''Initialize and then start ports search and autobaud process
        '''
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port, self.baud)
        self.open = self.ser.isOpen()
        self.latest = []
        self.ready = False
        self.pipe = pipe
        # self.header = A2_header     # packet type hex, default A2
        self.size = 0
        self.header = None
        self.parser = None
        if packet_type in packet_def.keys():
            self.size = packet_def[packet_type][0]
            self.header = packet_def[packet_type][1]
            self.parser = eval('self.parse_' + packet_type)
        else:
            self.open = False
            print('Unsupported packet type: %s'% packet_type)

    def start(self, reset=False, reset_cmd='5555725300FC88'):
        if self.open:
            bf = bytearray(self.size*2)
            n_bytes = 0
            if reset is True:
                self.ser.write(bytearray.fromhex(reset_cmd))
            self.ser.reset_input_buffer()
            while True:
                data = self.ser.read(self.size)
                ## parse new
                n = len(data)
                for i in range(n):
                    bf[n_bytes + i] = data[i]
                n_bytes += n
                while n_bytes >= self.size:
                    if bf[0] == packet_header[0] and bf[1] == packet_header[1] and\
                       bf[2] == self.header[0] and bf[3] == self.header[1]:
                        # crc
                        packet_crc = 256 * bf[self.size-2] + bf[self.size-1]
                        calculated_crc = self.calc_crc(bf[2:bf[4]+5])
                        # decode
                        if packet_crc == calculated_crc:
                            self.latest = self.parse_packet(bf[2:bf[4]+5])
                            # print(self.latest[0])
                            if self.pipe is not None:
                                self.pipe.send(self.latest)
                            # remove decoded data from the buffer
                            n_bytes -= self.size
                            for i in range(n_bytes):
                                bf[i] = bf[i+self.size]
                        else:
                            print('crc fail: %s %s %s %s'% (self.size, n_bytes, packet_crc, calculated_crc))
                            print('%s'% bf)
                            # remove the first byte from the buffer
                            n_bytes -= 1
                            for i in range(n_bytes):
                                bf[i] = bf[i+1]
                            n_bytes = self.sync_packet(bf, n_bytes, packet_header)
                    else:
                        n_bytes = self.sync_packet(bf, n_bytes, packet_header)
    def get_latest(self):
        return self.latest

    def parse_packet(self, payload):
        '''
        parse packet
        '''
        data = self.parser(payload[3::])
        return data

    def parse_S0(self, payload):
        '''S0 Payload Contents
        Byte Offset	Name	Format	Scaling	Units	Description
        0	xAccel	    I2	20/2^16	G	X accelerometer
        2	yAccel	    I2	20/2^16	G	Y accelerometer
        4	zAccel	    I2	20/2^16	G	Z accelerometer
        6	xRate   	I2	7*pi/2^16 [1260 deg/2^16]	rad/s [deg/sec]	X angular rate
        8	yRate	    I2	7*pi/2^16 [1260 deg/2^16]	rad/s [deg/sec]	Y angular rate
        10	zRate	    I2	7*pi/2^16 [1260 deg/2^16]	rad/s [deg/sec]	Z angular rate
        12	xMag	    I2	2/2^16	Gauss	X magnetometer
        14	yMag	    I2	2/2^16	Gauss	Y magnetometer
        16	zMag	    I2	2/2^16	Gauss	Z magnetometer
        18	xRateTemp	I2	200/2^16	deg. C	X rate temperature
        20	yRateTemp	I2	200/2^16	deg. C	Y rate temperature
        22	zRateTemp	I2	200/2^16	deg. C	Z rate temperature
        24	boardTemp	I2	200/2^16	deg. C	CPU board temperature
        26	GPSITOW	    U2	truncated	Ms	GPS ITOW (lower 2 bytes)
        28	BITstatus   U2 Master BIT and Status'''
        
        accels = [0 for x in range(3)] 
        for i in range(3):
            accel_int16 = (256 * payload[2*i] + payload[2*i+1]) - 65535 if 256 * payload[2*i] + payload[2*i+1] > 32767  else  256 * payload[2*i] + payload[2*i+1]
            accels[i] = (9.80665 * 20 * accel_int16) / math.pow(2,16)

        gyros = [0 for x in range(3)] 
        for i in range(3):
            gyro_int16 = (256 * payload[2*i+6] + payload[2*i+7]) - 65535 if 256 * payload[2*i+6] + payload[2*i+7] > 32767  else  256 * payload[2*i+6] + payload[2*i+7]
            gyros[i] = (1260 * gyro_int16) / math.pow(2,16) 

        mags = [0 for x in range(3)] 
        for i in range(3):
            mag_int16 = (256 * payload[2*i+12] + payload[2*i+13]) - 65535 if 256 * payload[2*i+12] + payload[2*i+13] > 32767  else  256 * payload[2*i+12] + payload[2*i+13]
            mags[i] = (2 * mag_int16) / math.pow(2,16) 

        temps = [0 for x in range(4)] 
        for i in range(4):
            temp_int16 = (256 * payload[2*i+18] + payload[2*i+19]) - 65535 if 256 * payload[2*i+18] + payload[2*i+19] > 32767  else  256 * payload[2*i+18] + payload[2*i+19]
            temps[i] = (200 * temp_int16) / math.pow(2,16)

        # Counter Value
        counter = 256 * payload[26] + payload[27]   

        # BIT Value
        bit = 256 * payload[28] + payload[29]

        return counter, accels, gyros, mags, temps, bit

    def parse_S1(self, payload):
        '''S1 Payload Contents
                Byte Offset	Name	Format	Scaling	Units	Description
                0	xAccel	I2	20/2^16	G	X accelerometer
                2	yAccel	I2	20/2^16	G	Y accelerometer
                4	zAccel	I2	20/2^16	G	Z accelerometer
                6	xRate	I2	7*pi/2^16   [1260 deg/2^16]	rad/s [deg/sec]	X angular rate
                8	yRate	I2	7*pi/2^16   [1260 deg/2^16]	rad/s [deg/sec]	Y angular rate
                10	zRate	I2	7*pi/2^16   [1260 deg/2^16]	rad/s [deg/sec]	Z angular rate
                12	xRateTemp	I2	200/2^16	deg. C	X rate temperature
                14	yRateTemp	I2	200/2^16	deg. C	Y rate temperature
                16	zRateTemp	I2	200/2^16	deg. C	Z rate temperature
                18	boardTemp	I2	200/2^16	deg. C	CPU board temperature
                20	counter         U2	-	packets	Output time stamp 
                22	BITstatus	U2	-	-	Master BIT and Status'''
        accels = [0 for x in range(3)] 
        for i in range(3):
            accel_int16 = (256 * payload[2*i] + payload[2*i+1]) - 65535 if 256 * payload[2*i] + payload[2*i+1] > 32767  else  256 * payload[2*i] + payload[2*i+1]
            accels[i] = (9.80665 * 20 * accel_int16) / math.pow(2,16)

        gyros = [0 for x in range(3)] 
        for i in range(3):
            gyro_int16 = (256 * payload[2*i+6] + payload[2*i+7]) - 65535 if 256 * payload[2*i+6] + payload[2*i+7] > 32767  else  256 * payload[2*i+6] + payload[2*i+7]
            gyros[i] = (1260 * gyro_int16) / math.pow(2,16) 

        temps = [0 for x in range(4)] 
        for i in range(4):
            temp_int16 = (256 * payload[2*i+12] + payload[2*i+13]) - 65535 if 256 * payload[2*i+12] + payload[2*i+13] > 32767  else  256 * payload[2*i+12] + payload[2*i+13]
            temps[i] = (200 * temp_int16) / math.pow(2,16)
    
        # Counter Value
        counter = 256 * payload[20] + payload[21]   

        # BIT Value
        bit = 256 * payload[22] + payload[23]         

        return counter, accels, gyros, temps, bit

    def parse_A1(self, payload):
        '''A1 Payload Contents
            0	rollAngle	I2	2*pi/2^16 [360 deg/2^16]	Radians [deg]	Roll angle
            2	pitchAngle	I2	2*pi/2^16 [360 deg/2^16]	Radians [deg]	Pitch angle
            4	yawAngleMag	I2	2*pi/2^16 [360 deg/2^16]	Radians [deg]	Yaw angle (magnetic north)
            6	xRateCorrected	I2	7*pi/2^16[1260 deg/2^16]	rad/s  [deg/sec]	X angular rate Corrected
            8	yRateCorrected	I2	7*pi/2^16 [1260 deg/2^16]	rad/s  [deg/sec]	Y angular rate Corrected
            10	zRateCorrected	I2	7*pi/2^16 [1260 deg/2^16]	rad/s  [deg/sec]	Z angular rate Corrected
            12	xAccel	I2	20/2^16	g	X accelerometer
            14	yAccel	I2	20/2^16	g	Y accelerometer
            16	zAccel	I2	20/2^16	g	Z accelerometer
            18	xMag	I2	2/2^16	Gauss	X magnetometer
            20	yMag	I2	2/2^16	Gauss	Y magnetometer
            22	zMag	I2	2/2^16	Gauss	Z magnetometer
            24	xRateTemp	I2	200/2^16	Deg C	X rate temperature
            26	timeITOW	U4	1	ms	DMU ITOW (sync to GPS)
            30	BITstatus	U2	-	-	Master BIT and Status'''

        angles = [0 for x in range(3)] 
        for i in range(3):
            angle_int16 = (256 * payload[2*i] + payload[2*i+1]) - 65535 if 256 * payload[2*i] + payload[2*i+1] > 32767  else  256 * payload[2*i] + payload[2*i+1]
            angles[i] = (360.0 * angle_int16) / math.pow(2,16) 

        gyros = [0 for x in range(3)] 
        for i in range(3):
            gyro_int16 = (256 * payload[2*i+6] + payload[2*i+7]) - 65535 if 256 * payload[2*i+6] + payload[2*i+7] > 32767  else  256 * payload[2*i+6] + payload[2*i+7]
            gyros[i] = (1260 * gyro_int16) / math.pow(2,16) 

        accels = [0 for x in range(3)] 
        for i in range(3):
            accel_int16 = (256 * payload[2*i+12] + payload[2*i+13]) - 65535 if 256 * payload[2*i+12] + payload[2*i+13] > 32767  else  256 * payload[2*i+12] + payload[2*i+13]
            accels[i] = (9.80665 * 20 * accel_int16) / math.pow(2,16)
        
        mags = [0 for x in range(3)] 
        for i in range(3):
            mag_int16 = (256 * payload[2*i+18] + payload[2*i+19]) - 65535 if 256 * payload[2*i+18] + payload[2*i+19] > 32767  else  256 * payload[2*i+18] + payload[2*i+19]
            mags[i] = (2 * mag_int16) / math.pow(2,16) 

        temp = [0 for x in range(3)] # compatible with A2
        temp_int16 = (256 * payload[2*i+24] + payload[2*i+25]) - 65535 if 256 * payload[2*i+24] + payload[2*i+25] > 32767  else  256 * payload[2*i+24] + payload[2*i+25]
        temp[0] = (200 * temp_int16) / math.pow(2,16)
    
        # Counter Value
        itow = 16777216 * payload[26] + 65536 * payload[27] + 256 * payload[28] + payload[29]   

        # BIT Value
        bit = 256 * payload[30] + payload[31]
            
        return angles, gyros, accels, temp, itow, bit

    def parse_A2(self, payload):
        '''A2 Payload Contents
        0	rollAngle	I2	2*pi/2^16 [360 deg/2^16]	Radians [deg]	Roll angle
        2	pitchAngle	I2	2*pi/2^16 [360 deg/2^16]	Radians [deg]	Pitch angle
        4	yawAngleMag	I2	2*pi/2^16 [360 deg/2^16]	Radians [deg]	Yaw angle (magnetic north)
        6	xRateCorrected	I2	7*pi/2^16[1260 deg/2^16]	rad/s  [deg/sec]	X angular rate Corrected
        8	yRateCorrected	I2	7*pi/2^16 [1260 deg/2^16]	rad/s  [deg/sec]	Y angular rate Corrected
        10	zRateCorrected	I2	7*pi/2^16 [1260 deg/2^16]	rad/s  [deg/sec]	Z angular rate Corrected
        12	xAccel	  I2	20/2^16	g	X accelerometer
        14	yAccel	  I2	20/2^16	g	Y accelerometer
        16	zAccel	  I2	20/2^16	g	Z accelerometer
        18	xRateTemp I2	200/2^16	Deg.C   X rate temperature 
        20	yRatetemp I2	200/2^16	Deg.C	Y rate temperature 
        22	zRateTemp I2	200/2^16	Deg.C   Z rate temperature 
        24	timeITOW	U4	1	ms	DMU ITOW (sync to GPS)
        28	BITstatus	U2	-	-	Master BIT and Status'''

        angles = [0 for x in range(3)] 
        for i in range(3):
            angle_int16 = (256 * payload[2*i] + payload[2*i+1]) - 65535 if 256 * payload[2*i] + payload[2*i+1] > 32767  else  256 * payload[2*i] + payload[2*i+1]
            angles[i] = (360.0 * angle_int16) / math.pow(2,16) 

        gyros = [0 for x in range(3)] 
        for i in range(3):
            gyro_int16 = (256 * payload[2*i+6] + payload[2*i+7]) - 65535 if 256 * payload[2*i+6] + payload[2*i+7] > 32767  else  256 * payload[2*i+6] + payload[2*i+7]
            gyros[i] = (1260 * gyro_int16) / math.pow(2,16) 

        accels = [0 for x in range(3)] 
        for i in range(3):
            accel_int16 = (256 * payload[2*i+12] + payload[2*i+13]) - 65535 if 256 * payload[2*i+12] + payload[2*i+13] > 32767  else  256 * payload[2*i+12] + payload[2*i+13]
            accels[i] = (9.80665 * 20 * accel_int16) / math.pow(2,16)

        temp = [0 for x in range(3)] 
        for i in range(3):
            temp_int16 = (256 * payload[2*i+18] + payload[2*i+19]) - 65535 if 256 * payload[2*i+18] + payload[2*i+19] > 32767  else  256 * payload[2*i+18] + payload[2*i+19]
            temp[i] = (200 * temp_int16) / math.pow(2,16)

        # Counter Value
        itow = 16777216 * payload[24] + 65536 * payload[25] + 256 * payload[26] + payload[27]   

        # BIT Value
        bit = 256 * payload[28] + payload[29]

        return angles, gyros, accels, temp, itow, bit

    def parse_z1(self, payload):
        '''
        parse z1 packet
        '''
        fmt = '=Ifffffffff'
        data = struct.unpack(fmt, payload)
        timer = data[0]
        acc = data[1:4]
        gyro = data[4:7]
        return timer, acc, gyro

    def parse_id(self, payload):
        '''
        parse id packet.
        The payload length (NumOfBytes) is based on the following:
            1 uint32_t (4 bytes) =   4 bytes   timer
            1 float  (4 bytes)   =   4 bytes   GPS heading
            1 uint32_t (4 bytes) =   4 bytes   GPS itow
            3 floats (4 bytes)   =  12 bytes   ea
            3 floats (4 bytes)   =  12 bytes   a
            3 floats (4 bytes)   =  12 bytes   aBias
            3 floats (4 bytes)   =  12 bytes   w
            3 floats (4 bytes)   =  12 bytes   wBias
            3 floats (4 bytes)   =  12 bytes   v
            3 floats (4 bytes)   =  12 bytes   GPS NED velocity
            3 double (8 bytes)   =  24 bytes   lla
            3 double (8 bytes)   =  24 bytes   gps LLA
            1 uint8_t (1 byte)   =   1 bytes
            1 uint8_t (1 byte)   =   1 bytes
            1 uint8_t (1 byte)   =   1 bytes
            =================================
                        NumOfBytes = 147 bytes
        '''
        fmt = '=I'          # timer
        fmt += 'f'          # GPS heading
        fmt += 'I'          # GPS itow
        fmt += 'fff'        # Euler angles
        fmt += 'fff'        # accel
        fmt += 'fff'        # accel bias, replaced by hdop, hacc, vacc for debug
        fmt += 'fff'        # gyro
        fmt += 'fff'        # gyro bias
        fmt += 'fff'        # velocity
        fmt += 'fff'        # GPS NED velocity
        fmt += 'ddd'        # lla
        fmt += 'ddd'        # debug
        fmt += 'B'          # opMode
        fmt += 'B'          # linAccelSw
        fmt += 'B'          # turnSw (bit1) gpsMeasurementUpdate (bit0)
        data = struct.unpack(fmt, payload)
        timer = data[0]
        gps_heading = data[1]
        gps_itow = data[2]
        euler = data[3:6]
        acc = data[6:9]
        acc_bias = data[9:12]
        gyro = data[12:15]
        gyro_bias = data[15:18]
        velocity = data[18:21]
        gps_velocity = data[21:24]
        lla = data[24:27]
        gps_lla = data[27:30]
        op_mode = data[30]
        lin_accel_sw = data[31] # replaced with num of satellites
        turn_sw = data[32]      # replaced with turnSw, pps, fix_type and gps update
        return timer, gps_itow, acc, gyro, lla, velocity, euler,\
            gps_lla, gps_velocity, gps_heading, acc_bias, turn_sw, lin_accel_sw

    def parse_e1(self, payload):
        '''
        parse e1 packet.
        The payload length (NumOfBytes) is based on the following:
                1 uint32_t (4 bytes) =   4 bytes    tstmp
                1 double (8 bytes)   =   8 bytes    dbTstmp (double)
                1 floats (4 bytes)   =  4 bytes     roll [deg]
                1 floats (4 bytes)   =  4 bytes     pitch [deg]
                1 floats (4 bytes)   =  4 bytes     yaw [deg]
                3 floats (4 bytes)   =  12 bytes    acc [g]
                3 floats (4 bytes)   =  12 bytes    gyro [dps]
                3 floats (4 bytes)   =  12 bytes    gyro bias [dps]
                3 floats (4 bytes)   =  12 bytes    m [Gauss]
                1 uint8_t (1 byte)   =   1 bytes    opMode
                1 uint8_t (1 byte)   =   1 bytes    accelLinSwitch
                1 uint8_t (1 byte)   =   1 bytes    turnSwitch
                =================================
                          NumOfBytes =  75 bytes
        '''
        fmt = '=I'          # timer
        fmt += 'd'          # time double
        fmt += 'fff'        # Euler angles
        fmt += 'fff'        # accel
        fmt += 'fff'        # gyro
        fmt += 'fff'        # gyro bias
        fmt += 'fff'        # mag
        fmt += 'B'          # opMode
        fmt += 'B'          # linAccelSw
        fmt += 'B'          # turnSw
        data = struct.unpack(fmt, payload)
        timer = data[0]
        euler = data[2:5]
        acc = data[5:8]
        gyro = data[8:11]
        gyro_bias = data[11:14]
        mag = data[14:17]
        op_mode = data[17]
        lin_accel_sw = data[18]
        turn_sw = data[19]
        return timer, euler, gyro, acc, turn_sw, lin_accel_sw

    def parse_e2(self, payload):
        '''
        parse e2 packet.
        The payload length (NumOfBytes) is based on the following:
                1 uint32_t (4 bytes) =   4 bytes   timer
                1 double (8 bytes)   =   8 bytes   timer(double)
                3 floats (4 bytes)   =  12 bytes   ea
                3 floats (4 bytes)   =  12 bytes   a
                3 floats (4 bytes)   =  12 bytes   aBias
                3 floats (4 bytes)   =  12 bytes   w
                3 floats (4 bytes)   =  12 bytes   wBias
                3 floats (4 bytes)   =  12 bytes   v
                3 floats (4 bytes)   =  12 bytes   m
                3 double (8 bytes)   =  24 bytes   lla
                1 uint8_t (1 byte)   =   1 bytes
                1 uint8_t (1 byte)   =   1 bytes
                1 uint8_t (1 byte)   =   1 bytes
                =================================
                          NumOfBytes = 123 bytes
        '''
        fmt = '=I'          # timer
        fmt += 'd'          # time double
        fmt += 'fff'        # Euler angles
        fmt += 'fff'        # accel
        fmt += 'fff'        # accel bias
        fmt += 'fff'        # gyro
        fmt += 'fff'        # gyro bias
        fmt += 'fff'        # velocity
        fmt += 'fff'        # mag
        fmt += 'ddd'        # lla
        fmt += 'B'          # opMode
        fmt += 'B'          # linAccelSw
        fmt += 'B'          # turnSw
        data = struct.unpack(fmt, payload)
        timer = data[0]
        euler = data[2:5]
        acc = data[5:8]
        acc_bias = data[8:11]
        gyro = data[11:14]
        gyro_bias = data[14:17]
        velocity = data[17:20]
        mag = data[20:23]
        lla = data[23:26]
        op_mode = data[26]
        lin_accel_sw = data[27]
        turn_sw = data[28]
        return timer, 0, acc, gyro, lla, velocity, euler,\
            (0,0,0), (0,0,0), 0, acc_bias, turn_sw, lin_accel_sw

    def parse_a2(self, payload):
        #   1 uint32_t (4 bytes) = 4 bytes,     itow
        #   1 double   (8 bytes) = 8 bytes,     itow
        #   3 floats   (4 bytes) = 12 bytes,    ypr, deg
        #   3 floats   (4 bytes) = 12 bytes,    corrected gyro, deg/s
        #   3 floats   (4 bytes) = 12 bytes,    corrected accel, m/s/s
        #  =================================
        #             NumOfBytes = 48 bytes
        fmt = '=I'          # itow
        fmt += 'd'          # itow, double
        fmt += 'fff'        # ypr
        fmt += 'fff'        # corrected gyro
        fmt += 'fff'        # corrected accel
        data = struct.unpack(fmt, payload)
        itow = data[0]
        # double_itow = data[1]
        ypr = data[2:5]
        corrected_w = data[5:8]
        corrected_a = data[8:11]
        return ypr, corrected_w, corrected_a, itow

    def sync_packet(self, bf, bf_len, header):
        idx = -1
        while 1:
            idx = bf.find(header[0], idx+1, bf_len)
            # first byte of the header not found
            if idx < 0:
                bf_len = 0
                break
            # first byte of the header is found and there is enough bytes in buffer
            #   to match the header and packet type
            elif idx <= (bf_len-4):
                if bf[idx+1] == header[1] and\
                    bf[idx+2] == self.header[0] and bf[idx+3] == self.header[1]:
                    bf_len = bf_len - idx
                    for i in range(bf_len):
                        bf[i] = bf[i+idx]
                    break
                else:
                    continue
            # first byte of the header is found, but there is not enough bytes in buffer
            #   to match the header and packet type
            else:
                bf_len = bf_len - idx
                for i in range(bf_len):
                    bf[i] = bf[i+idx]
                break
        return bf_len

    def calc_crc(self, payload):
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

if __name__ == "__main__":
    port = 'COM7'
    baud = 115200
    unit = imu38x(port, baud, packet_type='a2', pipe=None)
    unit.start()
