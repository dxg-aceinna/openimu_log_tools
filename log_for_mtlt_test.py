import math
import serial
import serial.tools.list_ports
from multiprocessing import Process, Pipe, Array
import time
import struct
import numpy as np
import attitude
import openimu
import imu38x
import ins1000
import kml.dynamic_kml as kml
import post_proccess_for_ins_test

#### INS381
mtlt_01 = {'port':'COM30',\
            'baud':115200,\
            'packet_type':'A2',\
            'unit_type':'imu38x',\
            # 'orientation':'-y+x+z',\
            'enable':True}

mtlt_02 = {'port':'COM11',\
            'baud':115200,\
            'packet_type':'A2',\
            'unit_type':'imu38x',\
            # 'orientation':'-y+x+z',\
            'enable':True}

mtlt_03 = {'port':'COM7',\
            'baud':115200,\
            'packet_type':'A2',\
            'unit_type':'imu38x',\
            # 'orientation':'-y+x+z',\
            'enable':False}


log_dir = './log_data/'
log_file1 = '1.csv'
log_file2 = '2.csv'
log_file3 = '3.csv'

def log_imu38x(port, baud, packet, pipe):
    imu38x_unit = imu38x.imu38x(port, baud, packet_type=packet, pipe=pipe)
    imu38x_unit.start()

if __name__ == "__main__":
    print('%s is mtlt_01.' % mtlt_01['port'])
    print('%s is mtlt_02.' % mtlt_02['port'])
    print('%s is mtlt_03.' % mtlt_03['port'])

    #### create pipes
    if mtlt_01['enable']:
        print('connecting to 01...')
        parent_conn_1, child_conn_1 = Pipe()
        process_target = log_imu38x
        p1 = Process(target=process_target,\
                     args=(mtlt_01['port'], mtlt_01['baud'],\
                     mtlt_01['packet_type'], child_conn_1)
                    )
        p1.daemon = True
        p1.start()
    if mtlt_02['enable']:
        print('connecting to 02...')
        parent_conn_2, child_conn_2 = Pipe()
        process_target = log_imu38x
        p2 = Process(target=process_target,\
                     args=(mtlt_02['port'], mtlt_02['baud'],\
                     mtlt_02['packet_type'], child_conn_2)
                    )
        p2.daemon = True
        p2.start()
    if mtlt_03['enable']:
        print('connecting to 03...')
        parent_conn_3, child_conn_3 = Pipe()
        process_target = log_imu38x
        p3 = Process(target=process_target,\
                     args=(mtlt_03['port'], mtlt_03['baud'],\
                     mtlt_03['packet_type'], child_conn_3)
                    )
        p3.daemon = True
        p3.start()

    #### create log file
    headerline = "recv_interval (s), openimu timer,"
    headerline += "ax (m/s2), ay (m/s2), az (m/s2),"
    headerline += "wx (deg/s), wy (deg/s), wz (deg/s),"
    headerline += "roll (deg), pitch (deg), yaw (deg)\n"
    data_file1 = log_dir + log_file1
    f1 = open(data_file1, 'w+')
    f1.truncate()
    f1.write(headerline)
    f1.flush()
    data_file2 = log_dir + log_file2
    f2 = open(data_file2, 'w+')
    f2.truncate()
    f2.write(headerline)
    f2.flush()
    data_file3 = log_dir + log_file3
    f3 = open(data_file3, 'w+')
    f3.truncate()
    f3.write(headerline)
    f3.flush()

    #### start logging
    # start time, to calculate recv interval
    tstart = time.time()
    # data from INS381
    timer1 = 0
    acc1 = np.zeros((3,))
    gyro1 = np.zeros((3,))
    euler1 = np.zeros((3,))
    acc2 = np.zeros((3,))
    gyro2 = np.zeros((3,))
    euler2 = np.zeros((3,))
    acc3 = np.zeros((3,))
    gyro3 = np.zeros((3,))
    euler3 = np.zeros((3,))

    # logging
    fmt = "%f, %u, "                    # time_interval, packet timer
    fmt += "%f, %f, %f, %f, %f, %f, "   # ins381 acc and gyro
    fmt += "%f, %f, %f\n" # ins381 lla/vel/euler
    try:
        while True:
            # 1. timer interval
            tnow = time.time()
            time_interval = tnow-tstart
            tstart = tnow
            # 2. INS381, timer, acc and gyro, lla, vel, Euler angles
            if mtlt_01['enable']:
                latest1 = parent_conn_1.recv()
                timer1 = latest1[4]
                acc1 = np.array(latest1[2])
                gyro1 = np.array(latest1[1])
                euler1 = np.array(latest1[0])
            if mtlt_02['enable']:
                latest2 = parent_conn_2.recv()
                acc2 = np.array(latest2[2])
                gyro2 = np.array(latest2[1])
                euler2 = np.array(latest2[0])
            if mtlt_03['enable']:
                latest3 = parent_conn_3.recv()
                acc3 = np.array(latest3[2])
                gyro3 = np.array(latest3[1])
                euler3 = np.array(latest3[0])

            # 5. log data to file
            lines = fmt% (\
                            time_interval, timer1,\
                            acc1[0], acc1[1], acc1[2],\
                            gyro1[0], gyro1[1], gyro1[2],\
                            euler1[0], euler1[1], euler1[2])
            f1.write(lines)
            f1.flush()
            lines = fmt% (\
                            time_interval, timer1,\
                            acc2[0], acc2[1], acc2[2],\
                            gyro2[0], gyro2[1], gyro2[2],\
                            euler2[0], euler2[1], euler2[2])
            f2.write(lines)
            f2.flush()
            lines = fmt% (\
                            time_interval, timer1,\
                            acc3[0], acc3[1], acc3[2],\
                            gyro3[0], gyro3[1], gyro3[2],\
                            euler3[0], euler3[1], euler3[2])
            f3.write(lines)
            f3.flush()
    except KeyboardInterrupt:
        print("Stop logging, preparing data for simulation...")
        f1.close()
        if mtlt_01['enable']:
            p1.terminate()
            p1.join()
        post_proccess_for_ins_test.post_processing(data_file1)
