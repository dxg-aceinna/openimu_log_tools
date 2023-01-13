import os
import sys
import math
import serial
import serial.tools.list_ports
import struct

preamble = bytearray.fromhex('5555')
# payload + 2-byte header + 2-byte type + 1-byte len + 2-byte crc
packet_def = {'A1': [39, bytearray.fromhex('4131')],\
              'A2': [37, bytearray.fromhex('4132')],\
              'S0': [37, bytearray.fromhex('5330')],\
              'S1': [31, bytearray.fromhex('5331')],\
              'SH': [37, bytearray.fromhex('5348')],\
              'S2': [45, bytearray.fromhex('5332')],\
              'E3': [39, bytearray.fromhex('4533')],\
              'SA': [25, bytearray.fromhex('5341')],\
              'MG': [35, bytearray.fromhex('4D47')],\
              'z1': [47, bytearray.fromhex('7a31')],\
              's1': [59, bytearray.fromhex('7331')],\              
              'a2': [55, bytearray.fromhex('6132')],\
              'e1': [82, bytearray.fromhex('6531')],\
              'e2': [130, bytearray.fromhex('6532')],\
              'id': [154, bytearray.fromhex('6964')],\
              'sd': [57, bytearray.fromhex('7364')],\
              'FM': [125, bytearray.fromhex('464D')],\
              'SM': [97, bytearray.fromhex('534D')]}   # IMU340

class imu38x:
    def __init__(self, port, baud=115200, packet_type='A2', pipe=None):
        '''
        Initialize and then start ports search and autobaud process
        If baud <= 0, then port is actually a data file.
        '''
        self.port = port
        self.baud = baud
        # is file or serial port
        self.physical_port = True
        self.file_size = 0
        if baud > 0:
            self.ser = serial.Serial(self.port, self.baud)
            self.open = self.ser.isOpen()
        else:
            self.ser = open(port, 'rb')
            self.open = True
            self.physical_port = False
            self.file_size = os.path.getsize(port)
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
        # serial data buffer
        self.bf = bytearray(self.size*2)
        self.nbf = 0    # how bytes in self.bf

    def start(self, reset=False, reset_cmd='5555725300FC88'):
        if self.open:
            # send optional reset command if port is a pysical serial port
            if self.physical_port:
                if reset is True:
                    self.ser.write(bytearray.fromhex(reset_cmd))
                self.ser.reset_input_buffer()
            while True:
                if self.physical_port:
                    read_size = self.ser.in_waiting
                else:
                    read_size = self.file_size
                data = self.ser.read(read_size)
                if not data:
                    # end processing if reaching the end of the data file
                    if not self.physical_port:
                        break
                else:
                    # parse new coming data
                    self.parse_new_data(data)
            #close port or file
            self.ser.close()
            # print('End of processing.')
            if self.pipe is not None:
                self.pipe.send('exit')

    def parse_new_data(self, data):
        '''
        add new data in the buffer
        '''
        n = len(data)
        for i in range(n):
            self.bf[self.nbf] = data[i]
            self.nbf += 1
            while self.nbf >= self.size:
                if self.bf[0] == preamble[0] and self.bf[1] == preamble[1] and\
                    self.bf[2] == self.header[0] and self.bf[3] == self.header[1]:
                    # crc
                    packet_crc = 256 * self.bf[self.size-2] + self.bf[self.size-1]
                    calculated_crc = self.calc_crc(self.bf[2:self.bf[4]+5])
                    # decode
                    if packet_crc == calculated_crc:
                        self.latest = self.parse_packet(self.bf[2:self.bf[4]+5])
                        print(*self.latest, sep=", ")
                        if self.pipe is not None:
                            self.pipe.send(self.latest)
                        # remove decoded data from the buffer
                        self.nbf -= self.size
                        for i in range(self.nbf):
                            self.bf[i] = self.bf[i+self.size]
                    else:
                        print('crc fail: %s %s %s %s'% (self.size, self.nbf, packet_crc, calculated_crc))
                        print(" ".join("{:02X}".format(self.bf[i]) for i in range(0, self.nbf)))
                        # remove the first byte from the buffer
                        self.nbf -= 1
                        for i in range(self.nbf):
                            self.bf[i] = self.bf[i+1]
                        self.nbf = self.sync_packet(self.bf, self.nbf, preamble)
                else:
                    self.nbf = self.sync_packet(self.bf, self.nbf, preamble)

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

        # return counter, accels, gyros, temps, bit
        rtn = []
        rtn.extend([counter])
        rtn.extend(accels)
        rtn.extend(gyros)
        rtn.extend(temps)
        rtn.extend([bit])
        return rtn

    def parse_SH(self, payload):
        '''SH Payload Contents
            0 	xAccel 	I4 	4*10^6	LSB/G	X accelerometer 
            4	yAccel 	I4 	4*10^6	LSB/G	Y accelerometer 33
            8 	zAccel 	I4 	4*10^6	LSB/G	Z accelerometer 
            12 	xRate	I4 	2.56*10^5	LSB/dps 	X angular rate 
            16	yRate 	I4 	2.56*10^5	LSB/dps	Y angular rate 
            20 	zRate 	I4 	2.56*10^5	LSB/dps	Z angular rate 
            24	Temp	I2 	400/2^16	deg. C	Temperature  
            26	Rolling Over counter 	U2 	- 	counts	Rolling Over counter 65536 counts/second
            28	BIT Status 	U2 	- 	bitmask	Master BIT status word
        '''
        accels = [0 for x in range(3)] 
        for i in range(3):
            accel_int16 = (16777216 * payload[4*i] + 65536 * payload[4*i+1] + 256 * payload[4*i+2] + payload[4*i+3]) - 4294967295 if (16777216 * payload[4*i] + 65536 * payload[4*i+1] + 256 * payload[4*i+2] + payload[4*i+3]) > 2147483647  else (16777216 * payload[4*i] + 65536 * payload[4*i+1] + 256 * payload[4*i+2] + payload[4*i+3])
            accels[i] = (9.80665 * accel_int16) / (4 * math.pow(10, 6))

        gyros = [0 for x in range(3)] 
        for i in range(3):
            gyro_int16 = (16777216 * payload[4*i+12] + 65536 * payload[4*i+13] + 256 * payload[4*i+14] + payload[4*i+15]) - 4294967295 if (16777216 * payload[4*i+12] + 65536 * payload[4*i+13] + 256 * payload[4*i+14] + payload[4*i+15]) > 2147483647  else (16777216 * payload[4*i+12] + 65536 * payload[4*i+13] + 256 * payload[4*i+14] + payload[4*i+15])
            gyros[i] = gyro_int16 / (2.56 * math.pow(10, 5)) 
        
        temps = [0 for x in range(1)] 
        for i in range(1):
            temp_int16 = (256 * payload[4*i+24] + payload[4*i+25]) - 65535 if 256 * payload[4*i+24] > 32767  else 256 * payload[4*i+24]
            temps[i] = (400 * temp_int16) / math.pow(2,16)
    
        # Counter Value
        counter = 256 * payload[26] + payload[27]   

        # BIT Value
        bit = 256 * payload[28] + payload[29]         

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

    def parse_s1(self, payload):
        '''
        parse s1 packet
        '''
        fmt = '=IQffffffffff'
        data = struct.unpack(fmt, payload)
        timer = data[0]
        acc = data[2:5]
        gyro = data[5:8]
        temp = data[11]
        return timer, acc, gyro, temp

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

    def parse_E3(self, payload):
        '''
        Byte Offset Name 	        Format 	Notes 	    Scaling 	unit 	Description 
                0 	Counter         U4 	    MSB first 	1 	        ms
                4 	Roll 	        I2 	    MSB first 	360° /2^16 	° 	 
                6 	Pitch 	        I2 	    MSB first 	360° /2^16 	° 	 
                8 	Yaw 	        I2 	    MSB first 	360° /2^16 	° 	 
                10 	Steering angle 	I2 	    MSB first 	360° /2^16	         Steering angle of front wheel
                12 	Accel_X_Master 	I2 	    MSB first 	20/2^16 	g 	 
                14 	Accel_Y_Master 	I2 	    MSB first 	20/2^16 	g 	 
                16 	Accel_Z_Master 	I2 	    MSB first 	20/2^16 	g 	 
                18 	Gyro_X_Master 	I2 	    MSB first 	1260°/2^16 	°/sec 	 
                20 	Gyro_Y_Master 	I2 	    MSB first 	1260°/2^16 	°/sec 	 
                22 	Gyro_Z_Master 	I2 	    MSB first 	1260°/2^16 	°/sec 	 
                24 	steering angle 	I2 	    MSB first 	1260°/2^16 	°/sec 	Gyro data of Z axis in slave IMU (when steering angle around Z axis); 
                    rate                                                    Rotation rate of steering angle. 
                26 	Vehicle speed 	I2 	    MSB first 	0.001 	    m/s 	Positive and negative value to show speed of advancing or retreating will be better 
                28 	Reserved 	    4 bytes MSB first 	 	 	            Reserved for future. 
        '''
        pow_2_16 = math.pow(2, 16)
        # Counter Value
        counter = struct.unpack('>I', payload[0:4])[0]
        # roll
        roll = struct.unpack('>h', payload[4:6])[0] * 360 / pow_2_16
        # pitch
        pitch = struct.unpack('>h', payload[6:8])[0] * 360 / pow_2_16
        # yaw
        yaw = struct.unpack('>h', payload[8:10])[0] * 360 / pow_2_16
        # steering angle
        steering_angle = struct.unpack('>h', payload[10:12])[0] * 360 / pow_2_16
        # acc master
        acc_master = [20/pow_2_16 * x for x in struct.unpack('>hhh', payload[12:18])]
        # gyro master
        gyro_master = [1260/pow_2_16 * x for x in struct.unpack('>hhh', payload[18:24])]
        # steering angle rate
        steering_angle_rate = struct.unpack('>h', payload[24:26])[0] * 1260 / pow_2_16
        # vehicle speed
        vehicle_speed = struct.unpack('>h', payload[26:28])[0] * 0.001
        # INS states
        ins_states = payload[28:30]
        # steering angle states
        sa_states = payload[30:32]
        print(['E3', gyro_master])
        return counter, [roll, pitch, yaw], [steering_angle, steering_angle_rate], vehicle_speed,\
               acc_master, gyro_master

    def parse_MG(self, payload):
        '''
        Byte Offset 	Name 	Format 	Notes 	Scaling 	unit 	Description 
        0 	Counter 	U4 	MSB first 	1 	ms 	Unsigned int, 4 bytes 
        4 	Accel_X_Master 	I2 	MSB first 	20/2^16 	g 	No used in the algorithm. Reserved here for possible future use. 
        6 	Accel_Y_Master 	I2 	MSB first 	20/2^16 	g 	No used in the algorithm. Reserved here for possible future use. 
        8 	Accel_Z_Master 	I2 	MSB first 	20/2^16 	g 	No used in the algorithm. Reserved here for possible future use. 
        10 	Gyro_X_Master 	I2 	MSB first 	1260°/2^16 	°/sec 	 
        12 	Gyro_Y_Master 	I2 	MSB first 	1260°/2^16 	°/sec 	 
        14 	Gyro_Z_Master 	I2 	MSB first 	1260°/2^16 	°/sec 	 
        16 	GNSS time of week 	U4 	MSB first 	 	ms 	This value can be acquired from the master GNSS driver. 
        20 	Ground speed 	I2 	MSB first 	0.001 	m/s 	Signed. The customer defines a speed limit of [-100, 100]km/h, which can be covered by the 16bit signed integer with a scaling of 0.001m/s. The speed accuracy is 0.03~0.05m/s, which can also be well handled by the scaling. This value can be acquired from the master GNSS driver. 
        22 	GNSS update flag 	Char 	 	 	 	No-zero value indicates the GNSS info is updated in this packet. 
        23 	GNSS fix type 	Char 	 	 	 	Zero indicates invalid GNSS info. This value can be acquired from the master GNSS driver. 
        24 	Reserved 	4 bytes 	MSB first 	 	 	Reserved for future use. 
        '''
        pow_2_16 = math.pow(2, 16)
        # Counter Value
        counter = struct.unpack('>I', payload[0:4])[0]
        # acc master
        acc_master = [20/pow_2_16 * x for x in struct.unpack('>hhh', payload[4:10])]
        # gyro master
        gyro_master = [1260/pow_2_16 * x for x in struct.unpack('>hhh', payload[10:16])]
        # time of week
        tow = struct.unpack('>I', payload[16:20])[0]
        # ground speed
        ground_speed = struct.unpack('>h', payload[20:22])[0] * 0.001
        # gnss update flag
        gnss_states = struct.unpack('bb', payload[22:24])
        gnss_update = gnss_states[0]
        # gnss fix type
        gnss_fix_type = gnss_states[1]
        # reserved
        print(['MG', tow, ground_speed, gnss_update, gnss_fix_type])
        return counter, acc_master, gyro_master

    def parse_SA(self, payload):
        '''
        Byte Offset 	Name 	Format 	Notes 	Scaling 	unit 	Description 
        0 	Counter 	U4 	MSB first 	1 	ms 	Unsigned int, 4 bytes 
        4 	Steering angle 	I2 	MSB first 	360° /2^16 	° 	Steering angle of front wheel 
        6 	Steering angle rate 	I2 	MSB first 	1260°/2^16 	°/sec 	Steering angle rate of front wheel 
        8 	Algorithm states 	U2 	MSB first 	 	 	Refer to Figure 1 for details 
        10 	Reserved 	8 bytes 	MSB first 	 	 	Reserved for future use. 
        '''
        pow_2_16 = math.pow(2, 16)
        # Counter Value
        counter = struct.unpack('>I', payload[0:4])[0]
        # steering angle
        steering_angle = struct.unpack('>h', payload[4:6])[0] * 360 / pow_2_16
        # steering angle rate
        steering_angle_rate = struct.unpack('>h', payload[6:8])[0] * 1260 / pow_2_16
        # algorihtm states
        steering_states = payload[8:10]
        print(['SA', steering_states])
        # reserved
        return counter, steering_angle, steering_angle_rate, steering_states

    def parse_sd(self, payload):
        '''
        parse sd packet.
        The payload length (NumOfBytes) is based on the following:
            // 1 uint32_t   (4 bytes)   =   4 bytes     timer
            // 3 floats     (4 bytes)   =   12 bytes    master gyro
            // 3 floats     (4 bytes)   =   12 bytes    master accel
            // 3 floats     (4 bytes)   =   12 bytes    slave gyro
            // 1 float      (4 byte)    =   4 byte      GNSS raw ground speed
            // 1 uint8_t    (1 byte)    =   1 byte      GNSS update flag
            // 1 uint8_t    (1 byte)    =   1 byte      GNSS fix type
            // 1 uint32_t   (4 byte)    =   4 byte      GNSS TOW
            // =================================
            //              NumOfBytes  =  54 bytes
        '''
        fmt = '=I'          # timer
        fmt += 'fff'        # master gyro
        fmt += 'fff'        # master accel
        fmt += 'fff'        # slave gyro
        fmt += 'f'          # ground speed
        fmt += 'b'          # GNSS update flag
        fmt += 'b'          # GNSS fix type
        fmt += 'I'          # GNSS TOW

        data = struct.unpack(fmt, payload)
        timer = data[0]
        w_master = data[1:4]
        a_master = data[4:7]
        w_slave = data[7:10]
        ground_speed = data[10]
        update_flag = data[11]
        fix_type = data[12]
        gps_itow = data[13]
        # print([timer, gps_itow, w_master, a_master, w_slave, ground_speed, update_flag, fix_type])
        return timer, gps_itow, w_master, a_master, w_slave, ground_speed, update_flag, fix_type

    def parse_FM(self, payload):
        '''
        Byte Offset 	Name 	Format 	Notes 	Scaling 	unit 	Description 
        0 	xAccelCounts1 	I4 	- 	counts 	Ux accelerometer (Chip#= sensorSubset *4)
        4 	yAccelCounts1 	I4 	- 	counts 	Uy accelerometer (Chip#= sensorSubset *4)
        8 	zAccelCounts1 	I4 	- 	counts 	Uz accelerometer (Chip#= sensorSubset *4)
        12 	xRateCounts1 	I4 	- 	counts 	Ux angular rate (Chip#= sensorSubset *4)
        16 	yRateCounts1 	I4 	- 	counts 	Uy angular rate (Chip#= sensorSubset *4)
        20 	zRateCounts1 	I4 	- 	counts 	Uz angular rate (Chip#= sensorSubset *4)
        24 	TempCounts1	    I4 	- 	counts 	Temperature  (Chip#= sensorSubset *4)
        28 	xAccelCounts2 	I4 	- 	counts 	Ux accelerometer  (Chip#= sensorSubset *4+1)
        32	yAccelCounts2 	I4 	- 	counts 	Uy accelerometer  (Chip#= sensorSubset *4+1)
        36	zAccelCounts2 	I4 	- 	counts 	Uz accelerometer  (Chip#= sensorSubset *4+1)
        40 	xRateCounts2	I4 	- 	counts 	Ux angular rate  (Chip#= sensorSubset *4+1)
        44 	yRateCounts2 	I4 	- 	counts 	Uy angular rate  (Chip#= sensorSubset *4+1)
        48 	zRateCounts2 	I4 	- 	counts 	Uz angular rate  (Chip#= sensorSubset *4+1)
        52 	TempCounts2	    I4 	- 	counts 	Temperature  (Chip#= sensorSubset *4+1)
        56	xAccelCounts3 	I4 	- 	counts 	Ux accelerometer  (Chip#= sensorSubset *4+2)
        60	yAccelCounts3 	I4 	- 	counts 	Uy accelerometer  (Chip#= sensorSubset *4+2)
        64	zAccelCounts3 	I4 	- 	counts 	Uz accelerometer  (Chip#= sensorSubset *4+2)
        68 	xRateCounts3 	I4 	- 	counts 	Ux angular rate  (Chip#= sensorSubset *4+2)
        72 	yRateCounts3 	I4 	- 	counts 	Uy angular rate  (Chip#= sensorSubset *4+2)
        76 	zRateCounts3 	I4 	- 	counts 	Uz angular rate  (Chip#= sensorSubset *4+2)
        80 	TempCounts3	    I4 	- 	counts 	Temperature  (Chip#= sensorSubset *4+2)
        84	    xAccel4	    F4	Float32	G	X accelerometer  Scaled
        88	    yAccel4 	F4	Float32	G 	Y accelerometer  Scaled
        92	    zAccel4 	F4	Float32	G 	Z accelerometer  Scaled
        96 	    xRate4	    F4	Float32	dps 	X angular rate     Scaled
        100 	yRate4 	    F4	Float32	rps(dps) 	Y angular rate     Scaled
        104	    zRate4 	    F4	Float32	rps(dps) 	Z angular rate     Scaled
        108	    Temp4	    F4	Float32	deg. C 	Temperature      Scaled
        112 	fragmentNum	U2 	- 	number	Always 0
        114	    packetIdx 	U2 	- 	number	Packet index
        116	    status	    U2			Master status



        84	xAccelCounts4 	I4 	- 	counts 	Ux accelerometer  (Chip#= sensorSubset *4+3)
        88	yAccelCounts4 	I4 	- 	counts 	Uy accelerometer  (Chip#= sensorSubset *4+3)
        92	zAccelCounts4 	I4 	- 	counts 	Uz accelerometer  (Chip#= sensorSubset *4+3)
        96	xRateCounts4 	I4 	- 	counts 	Ux angular rate  (Chip#= sensorSubset *4+3)
        100 yRateCounts4 	I4 	- 	counts 	Uy angular rate  (Chip#= sensorSubset *4+3)
        104	zRateCounts4 	I4 	- 	counts 	Uz angular rate  (Chip#= sensorSubset *4+3)
        108	TempCounts4	    I4 	- 	counts 	Temperature  (Chip#= sensorSubset *4+3)
        112 sensorSubset 	U2 	- 	number	Multiply by 4 to get first sensor chip number in the packet 
        114	sampleIdx 	    U2 	- 	number	Sample idx. Packets with the same sample idx present sensors data taken at the same moment of time. 
        '''
        fmt = 'i'*21   # four chips, 7 (3 accel, 3 gyo and 1 temp) for each
        fmt += 'f'*7
        fmt += 'H'*3
        data = struct.unpack(fmt, payload)
        # reserved
        return data

    def parse_SM(self, payload):
        '''
        IMU340
        Byte Offset 	Name 	Format 	Notes 	Scaling 	unit 	Description 
        0	xAccel1 	I2 	20/2^16	G	X accelerometer (Chip0)
        2	yAccel1 	I2 	20/2^16	G 	Y accelerometer (Chip0)
        4	zAccel1 	I2 	20/2^16	G 	Z accelerometer (Chip0)
        6	xRate1 	    I2 	1260°/2^16	dps 	X angular rate (Chip0)
        8	yRate1 	    I2 	1260°/2^16	dps 	Y angular rate (Chip0)
        10	zRate1 	    I2 	1260°/2^16	dps 	Z angular rate (Chip0)
        12	Temp1	    I2 	200/2^16	deg. C	Temperature  (Chip0)
        14	xAccel2 	I2 	20/2^16	G	X accelerometer (Chip1)
        16	yAccel2 	I2 	20/2^16	G	Y accelerometer  (Chip1)
        18	zAccel2 	I2 	20/2^16	G 	Z accelerometer  (Chip1)
        20	xRate2	    I2 	1260°/2^16	dps 	X angular rate  (Chip1)
        22	yRate2 	    I2 	1260°/2^16	dps 	Y angular rate  (Chip1)
        24	zRate2 	    I2 	1260°/2^16	dps 	Z angular rate  (Chip1)
        26	Temp2	    I2 	200/2^16	deg. C 	Temperature  (Chip1)
        28	xAccel3 	I2 	20/2^16	G	X accelerometer  (Chip2)
        30	yAccel3 	I2 	20/2^16	G 	Y accelerometer  (Chip2)
        32	zAccel3 	I2 	20/2^16	G 	Z accelerometer  (Chip2)
        34	xRate3 	    I2 	1260°/2^16	dps 	X angular rate  (Chip2)
        36	yRate3 	    I2 	1260°/2^16	dps 	Y angular rate  (Chip2)
        38	zRate3 	    I2 	1260°/2^16	dps 	Z angular rate  (Chip2)
        40	Temp3	    I2 	200/2^16 	deg. C 	Temperature  (Chip2)
        42	xAccel4	    I2 	20/2^16	G	X accelerometer  (Chip3)
        44	yAccel4 	I2 	20/2^16	G 	Y accelerometer  (Chip3)
        46	zAccel4 	I2 	20/2^16	G 	Z accelerometer  (Chip3)
        48	xRate4	    I2 	1260°/2^16	dps 	X angular rate (Chip3)
        50	yRate4 	    I2 	1260°/2^16	dps 	Y angular rate  (Chip3)
        52	zRate4 	    I2 	1260°/2^16	dps 	Z angular rate  (Chip3)
        54	Temp4	    I2 	200/2^16 	deg. C 	Temperature  (Chip3)
        56	xAccel5	    I2 	20/2^16	G	X accelerometer  (Chip4)
        58	yAccel5 	I2 	20/2^16	G 	Y accelerometer  (Chip4)
        60	zAccel5 	I2 	20/2^16	G 	Z accelerometer  (Chip4)
        62	xRate5	    I2 	1260°/2^16	dps 	X angular rate (Chip4)
        64	yRate5 	    I2 	1260°/2^16	dps 	Y angular rate  (Chip4)
        66	zRate5 	    I2 	1260°/2^16	dps 	Z angular rate  (Chip4)
        68	Temp5	    I2 	200/2^16 	deg. C 	Temperature  (Chip4)
        70	xAccel6	    I2 	20/2^16	G	X accelerometer  (Chip+)
        72	yAccel6 	I2 	20/2^16	G 	Y accelerometer  (Chip+)
        74	zAccel6 	I2 	20/2^16	G 	Z accelerometer  (Chip+)
        76	xRate6	    I2 	1260°/2^16	dps 	X angular rate (Chip+)
        78	yRate6 	    I2 	1260°/2^16	dps 	Y angular rate  (Chip+)
        80	zRate6 	    I2 	1260°/2^16	dps 	Z angular rate  (Chip+)
        82	Temp6	    I2 	200/2^16 	deg. C 	Temperature  (Chip+)
        84	sensorSubset 	U2 	- 	number	0
        86	sampleIdx 	    U2 	- 	number	Packet count 
        88	Master Bit Word	U2		bitmask	Master BIT word same as in S0/S1 packets
        '''
        fmt = '>'
        fmt += 'h'*42   # six chips, 7 (3 accel, 3 gyo and 1 temp) for each
        fmt += 'H'*3
        data = struct.unpack(fmt, payload)
        data = list(data)
        for ii in range(0, 42):
            tmp = ii % 7
            if tmp <= 2:
                data[ii] *= 20/pow(2, 16)
            elif tmp <= 5:
                data[ii] *= 1260/pow(2, 16)
            else:
                data[ii] *= 200/pow(2, 16)
        # reserved
        return data

    def parse_S2(self, payload):
        '''
        Byte Offset	Name	Format	Unit	Description
        0	gps_week	uint16		GPS week
        2	gps_millisecs	uint32	ms	GPS time of week. Effective sample instance is 1/(2*ODR) seconds earlier.
        6	accel_x	float	m/s^2	accel x axis measurement
        10	accel_y	float	m/s^2	accel y axis measurement
        14	accel_z	float	m/s^2	accel z axis measurement
        18	gyro_x	float	deg/s	gyro x axis measurement
        22	gyro_y	float	deg/s	gyro y axis measurement
        26	gyro_z	float	deg/s	gyro z axis measurement
        30	temperature	float 	deg C	Unit temperature
        34*	Master status word	uint32_t	bitmask	Device status word
        '''
        fmt = '<HI'
        fmt += 'f'*7
        fmt += 'I'
        data = struct.unpack(fmt, payload)
        # reserved
        return data

    def sync_packet(self, bf, bf_len, preamble):
        idx = -1
        while 1:
            idx = bf.find(preamble[0], idx+1, bf_len)
            # first byte of the header not found
            if idx < 0:
                bf_len = 0
                break
            # first byte of the header is found and there is enough bytes in buffer
            #   to match the header and packet type
            elif idx <= (bf_len-4):
                if bf[idx+1] == preamble[1] and\
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
    headerline_S1 = "time_stamp, acc_x (m/s2), acc_y (m/s2), acc_z (m/s2), gyro_x (dps),gyro_y (dps),gyro_z (dps),temp (degC),temp (degC),temp (degC),temp (degC),status"
    headerline_FM = "a0x,a0y,a0z,w0x,w0y,w0z,temp0,a1x,a1y,a1z,w1x,w1y,w1z,temp1,a2x,a2y,a2z,w2x,w2y,w2z,temp2,a3x,a3y,a3z,w3x,w3y,w3z,temp3,fragmentNum,packetIndex,status"
    headerline_SM = "xAccel1 ,yAccel1 ,zAccel1 ,xRate1 ,yRate1 ,zRate1 ,Temp1,xAccel2 ,yAccel2 ,zAccel2 ,xRate2,yRate2 ,zRate2 ,Temp2,xAccel3 ,yAccel3 ,zAccel3 ,xRate3 ,yRate3 ,zRate3 ,Temp3,xAccel4,yAccel4 ,zAccel4 ,xRate4,yRate4 ,zRate4 ,Temp4,xAccel5,yAccel5 ,zAccel5 ,xRate5,yRate5 ,zRate5 ,Temp5,xAccel6,yAccel6 ,zAccel6 ,xRate6,yRate6 ,zRate6 ,Temp6,sensorSubset ,sampleIdx ,Master Bit Word"
    headerline_S2 = "gps_week, gps_millisecs, accel_x (g), accel_y (g), accel_z (g), gyro_x (dps), gyro_y (dps), gyro_z (dps), temp (degC), master_status"
    # default settings
    port = 'd:\\MyDocuments\\desktop\\user_2023_01_05_14_44_56.bin'
    baud = 0
    packet_type = 'S2'
    # get settings from CLI
    num_of_args = len(sys.argv)
    if num_of_args > 1:
        port = sys.argv[1]
        if num_of_args > 2:
            baud = int(sys.argv[2])
            if num_of_args > 3:
                packet_type = sys.argv[3]
    # headerline
    if packet_type == "S1":
        print(headerline_S1)
    elif packet_type == "FM":
        print(headerline_FM)
    elif packet_type == "SM":
        print(headerline_SM)
    elif packet_type == "S2":
        print(headerline_S2)

    # run
    unit = imu38x(port, baud, packet_type, pipe=None)
    unit.start()
