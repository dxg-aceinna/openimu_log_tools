import math
import serial
import serial.tools.list_ports
import threading
from multiprocessing import Process, Pipe, Array
import time
import socket
import struct
import numpy as np
import attitude
import imu38x
import ins1000

a2_size = 37
nav_size = 127
enable_ref = False

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
PORT = 10600
network = '<broadcast>'

def log_new(port, baud, pipe):
    new_unit = imu38x.imu38x(port, baud, 'A1', pipe=pipe)
    new_unit.start()

def log_old(port, baud, pipe):
    old_unit = imu38x.imu38x(port, baud, pipe=pipe)
    old_unit.start()

def log_ref(port, baud, pipe):
    ref_unit = ins1000.ins1000(port, baud, pipe=pipe)
    ref_unit.start()

def get_com_ports():
    new_port = None
    old_port = None
    # automatically choose com ports
    while new_port is None or old_port is None:
        port_list = list(serial.tools.list_ports.comports())
        n = len(port_list)
        if n > 4:   # 4 for openimu evb, 1 for 380 adapter board
            port_names = []
            port_nums = []
            for i in range(n):
                this_name = list(port_list[i])[0]
                this_num = int(this_name[3::])
                port_names.append(this_name)
                port_nums.append(this_num)
            port_nums.sort()
            if (port_nums[0] + 1) in port_nums:
                new_port = 'COM' + str(port_nums[0])
                old_port = 'COM' + str(port_nums[-1])
            else:
                new_port = 'COM' + str(port_nums[1])
                old_port = 'COM' + str(port_nums[0])
    return new_port, old_port

if __name__ == "__main__":
    # find ports
    new_port = 'COM7'
    old_port = 'COM30'
    ref_port = None
    # if not enable_ref:
    #     [new_port, old_port] = get_com_ports()
    #     ref_port = None
    print('%s is the unit with the new algorthm' % new_port)
    print('%s is the unit with the old algorthm' % old_port)
    print('%s is the ref unit' % ref_port)    

    # create pipes
    parent_conn_new, child_conn_new = Pipe()
    parent_conn_old, child_conn_old = Pipe()
    # data

    p_new = Process(target=log_new, args=(new_port, 115200, child_conn_new))
    p_old = Process(target=log_old, args=(old_port, 115200, child_conn_old))
    p_new.daemon = True
    p_old.daemon = True
    p_new.start()
    p_old.start()
    if enable_ref:
        parent_conn_ref, child_conn_ref = Pipe()
        p_ref = Process(target=log_ref, args=(ref_port, 230400, child_conn_ref))
        p_ref.daemon = True
        p_ref.start()

    # create log file
    file = "log.txt"
    f = open(file, 'w+')
    f.truncate()

    # starting time
    tstart = time.time()

    # start logging
    latest_new = []
    latest_old = [[0.0, 0.0, 0.0]]
    latest_ref = np.zeros((3,))
    while True:
        # wait for data ready
        tnow = time.time()
        time_interval = tnow-tstart
        tstart = tnow
        # print(data_new[:])
        # time.sleep(1)
        latest_new = parent_conn_new.recv()
        latest_old = parent_conn_old.recv()
        if( enable_ref and parent_conn_ref.poll()):
            quat = parent_conn_ref.recv()[3]
            latest_ref = attitude.quat2euler(quat)
            latest_ref[0] = latest_ref[0] * attitude.R2D
            latest_ref[1] = latest_ref[1] * attitude.R2D
            latest_ref[2] = latest_ref[2] * attitude.R2D
            # print(latest_ref)
        # print(d1)
        lines = "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %e, %e, %e, %f, %f, %f, %f, %f, %f\n" % (\
                time_interval,\
                latest_new[0][0], latest_new[0][1], latest_new[0][2],\
                latest_old[0][0], latest_old[0][1], latest_old[0][2],\
                latest_ref[2], latest_ref[1], latest_ref[0],\
                latest_new[1][0], latest_new[1][1], latest_new[1][2],\
                latest_new[2][0], latest_new[2][1], latest_new[2][2],\
                latest_new[2][0], latest_new[2][1], latest_new[2][2],\
                latest_old[1][0], latest_old[1][1], latest_old[1][2],\
                latest_old[2][0], latest_old[2][1], latest_old[2][2])
        f.write(lines)
        f.flush()
        # udp
        packed_data = struct.pack('dddddddddd', latest_new[0][0], latest_new[0][1],\
                                    latest_old[0][0], latest_old[0][1],\
                                    latest_ref[2], latest_ref[1],\
                                    latest_new[2][0], latest_new[2][1], latest_new[2][2],\
                                    latest_new[2][0]
                                    )
        s.sendto(packed_data, (network, PORT))
