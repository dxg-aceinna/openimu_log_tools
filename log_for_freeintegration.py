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
import openimu
import ins1000
import post_proccess_for_free_integration

#### used to send out data via UDP
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
PORT = 10600
network = '<broadcast>'

#### INS1000 available?
enable_ref = True

def log_openimu(port, baud, pipe):
    imu = openimu.openimu(port, baud, pipe)
    imu.start()

def log_ins1000(port, baud, pipe):
    ins = ins1000.ins1000(port, baud, pipe)
    ins.start()

if __name__ == "__main__":
    #### find ports
    new_port = 'COM7'
    ref_port = 'COM15'
    if not enable_ref:
        ref_port = None
    print('%s is an OpenIMU.' % new_port)
    print('%s is an INS1000.' % ref_port)

    #### create pipes
    # imu
    parent_conn_openimu, child_conn_openimu = Pipe()
    p_openimu = Process(target=log_openimu, args=(new_port, 115200, child_conn_openimu))
    p_openimu.daemon = True
    p_openimu.start()
    # ins1000
    if enable_ref:
        parent_conn_ins1000, child_conn_ins1000 = Pipe()
        p_ins1000 = Process(target=log_ins1000, args=(ref_port, 230400, child_conn_ins1000))
        p_ins1000.daemon = True
        p_ins1000.start()

    #### create log file
    data_file = "log.txt"
    f = open(data_file, 'w+')
    f.truncate()
    headerline = "recv_interval (s), openimu timer,"
    headerline += "ax (m/s2), ay (m/s2), az (m/s2),"
    headerline += "wx (deg/s), wy (deg/s), wz (deg/s),"
    headerline += "Lat (deg), Lon (deg), Alt (m),"
    headerline += "vN (m/s), vE (m/s), vD (m/s),"
    headerline += "roll (deg), pitch (deg), yaw (deg)\n"
    f.write(headerline)
    f.flush()

    #### start logging
    # start time, to calculate recv interval
    tstart = time.time()
    # data from openimu and ins1000
    latest_openimu_timer = 0
    latest_openimu_acc = np.zeros((3,))
    latest_openimu_gyro = np.zeros((3,))
    latest_ref_lla = np.zeros((3,))
    latest_ref_vel = np.zeros((3,))
    latest_ref_euler = np.zeros((3,))
    # logging
    try:
        while True:
            # 1. timer interval
            tnow = time.time()
            time_interval = tnow-tstart
            tstart = tnow
            # 2. openimu timer, acc and gyro
            latest_openimu = parent_conn_openimu.recv()
            latest_openimu_timer = latest_openimu[0]
            latest_openimu_acc = latest_openimu[1]
            latest_openimu_gyro = latest_openimu[2]
            # 3. ins1000 time, lla, vel and quat
            if( enable_ref and parent_conn_ins1000.poll()):
                latest_ref = parent_conn_ins1000.recv()
                latest_ref_lla = latest_ref[1]
                latest_ref_vel = latest_ref[2]
                latest_ref_euler = attitude.quat2euler(latest_ref[3])   #ypr
                latest_ref_euler[0] = latest_ref_euler[0] * attitude.R2D
                latest_ref_euler[1] = latest_ref_euler[1] * attitude.R2D
                latest_ref_euler[2] = latest_ref_euler[2] * attitude.R2D
            # 4. log data to file
            lines = "%f, %u, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (\
                    time_interval, latest_openimu_timer,\
                    latest_openimu_acc[0], latest_openimu_acc[1], latest_openimu_acc[2],\
                    latest_openimu_gyro[0], latest_openimu_gyro[1], latest_openimu_gyro[2],\
                    latest_ref_lla[0], latest_ref_lla[1], latest_ref_lla[2],\
                    latest_ref_vel[0], latest_ref_vel[1], latest_ref_vel[2],
                    latest_ref_euler[2], latest_ref_euler[1], latest_ref_euler[0])
            f.write(lines)
            f.flush()
            # 5. send data via UDP
            packed_data = struct.pack('dddddd',\
                            latest_openimu_acc[0], latest_openimu_acc[1], latest_openimu_acc[2],\
                            latest_openimu_gyro[0], latest_openimu_gyro[1], latest_openimu_gyro[2])
            s.sendto(packed_data, (network, PORT))
    except KeyboardInterrupt:
        print("Stop logging, preparing data for simulation...")
        f.close()
        p_openimu.terminate()
        p_openimu.join()
        if enable_ref:
            p_ins1000.terminate()
            p_ins1000.join()
        post_proccess_for_free_integration.post_processing(data_file)
