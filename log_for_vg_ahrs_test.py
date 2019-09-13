import math
import serial
import serial.tools.list_ports
from multiprocessing import Process, Pipe, Array
import time
import struct
import numpy as np
import attitude
import imu38x
import socket

#### openimu
openimu_unit = {'port':'COM7',\
            'baud':230400,\
            'packet_type':'e1',\
            'unit_type':'imu38x',\
            # 'orientation':'-y+x+z',\
            'enable':True}

imu381_unit = {'port':'COM30',\
            'baud':115200,\
            'packet_type':'A2',\
            'unit_type':'imu38x',\
            # 'orientation':'-y+x+z',\
            'enable':True}

log_dir = './log_data/'
log_file = 'log.csv'


def log_imu38x(port, baud, packet, pipe):
    imu38x_unit = imu38x.imu38x(port, baud, packet_type=packet, pipe=pipe)
    imu38x_unit.start()

def orientation(data, ori):
    '''
    change coordiante according to ori.
    Args:
        data: nx3 numpy array
        ori: a string including +x, -x +y, -y, +z, -z, for example: '-y+x+z'
    Return:
        data after coordinate change
    '''
    map = {'+x': [0, 1],\
           '-x': [0, -1],\
           '+y': [1, 1],\
           '-y': [1, -1],\
           '+z': [2, 1],\
           '-z': [2, -1]}
    ori = ori.lower()
    idx_x = int(0)
    sgn_x = 1
    idx_y = int(1)
    sgn_y = 1
    idx_z = int(2)
    sgn_z = 1
    if len(ori) == 6:
        if ori[0:2] in map:
            idx_x = int(map[ori[0:2]][0])
            sgn_x = map[ori[0:2]][1]
        if ori[2:4] in map:
            idx_y = int(map[ori[2:4]][0])
            sgn_y = map[ori[2:4]][1]
        if ori[4:6] in map:
            idx_z = int(map[ori[4:6]][0])
            sgn_z = map[ori[4:6]][1]
        data[0], data[1], data[2] =\
            sgn_x * data[idx_x], sgn_y * data[idx_y], sgn_z * data[idx_z]
    return data

if __name__ == "__main__":
    # udp
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    PORT = 10600
    network = '<broadcast>'
    #### find ports
    if not openimu_unit['enable']:
        openimu_unit['port'] = None
    if not imu381_unit['enable']:
        imu381_unit['port'] = None
    print('%s is an OpenIMU.' % openimu_unit['port'])
    print('%s is an imu381.' % imu381_unit['port'])

    #### create pipes
    # openimu
    if openimu_unit['enable']:
        print('connecting to OpenIMIU...')
        parent_conn_openimu, child_conn_openimu = Pipe()
        process_target = log_imu38x
        p_openimu = Process(target=process_target,\
                            args=(openimu_unit['port'], openimu_unit['baud'],\
                                  openimu_unit['packet_type'], child_conn_openimu)
                           )
        p_openimu.daemon = True
        p_openimu.start()
    # imu381
    if imu381_unit['enable']:
        print('connecting to imu381...')
        parent_conn_imu381, child_conn_imu381 = Pipe()
        process_target = log_imu38x
        p_imu381 = Process(target=process_target,\
                           args=(imu381_unit['port'], imu381_unit['baud'],\
                           imu381_unit['packet_type'], child_conn_imu381)
                       )
        p_imu381.daemon = True
        p_imu381.start()

    #### create log file
    data_file = log_dir + log_file
    f = open(data_file, 'w+')
    f.truncate()
    headerline = "recv_interval (s), openimu timer,"
    headerline += "ax (m/s2), ay (m/s2), az (m/s2),"
    headerline += "wx (deg/s), wy (deg/s), wz (deg/s),"
    headerline += "roll (deg), pitch (deg), yaw (deg),"
    headerline += "ref_roll (deg), ref_pitch (deg), ref_yaw (deg)\n"
    f.write(headerline)
    f.flush()

    #### start logging
    # start time, to calculate recv interval
    tstart = time.time()
    # data from OpenIMU
    openimu_timer = 0
    openimu_acc = np.zeros((3,))
    openimu_gyro = np.zeros((3,))
    openimu_euler = np.zeros((3,))
    imu381_euler = np.zeros((3,))
    # logging
    try:
        while True:
            # 1. timer interval
            tnow = time.time()
            time_interval = tnow-tstart
            tstart = tnow
            # 2. openimu, timer, acc and gyro, Euler angles
            if openimu_unit['enable']:
                latest_openimu = parent_conn_openimu.recv()
                openimu_timer = latest_openimu[0]
                openimu_euler = np.array(latest_openimu[1])
                openimu_gyro = np.array(latest_openimu[2])
                openimu_acc = np.array(latest_openimu[3])
                if 'orientation' in openimu_unit:
                    openimu_acc = orientation(openimu_acc, openimu_unit['orientation'])
                    openimu_gyro = orientation(openimu_gyro, openimu_unit['orientation'])
            if imu381_unit['enable']:
                latest_imu381 = None
                while parent_conn_imu381.poll():
                    latest_imu381 = parent_conn_imu381.recv()
                # imu381_timer = latest_imu381[0]
                # imu381_acc = np.array(latest_imu381[3])
                # imu381_gyro = np.array(latest_imu381[2])
                if latest_imu381 is not None:
                    imu381_euler = np.array(latest_imu381[0])
                # if 'orientation' in openimu_unit:
                #     imu381_acc = orientation(imu381_acc, imu381_unit['orientation'])
                #     imu381_gyro = orientation(imu381_gyro, imu381_unit['orientation'])
            # 3. log data to file
            fmt = "%f, %u, "                    # itow, packet timer
            fmt += "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, "   # openimu acc and gyro
            fmt += "%f, %f, %f, %f, %f, %f\n" # openimu and imu381 Euler angles
            lines = fmt% (\
                            time_interval, openimu_timer,\
                            openimu_acc[0], openimu_acc[1], openimu_acc[2],\
                            openimu_gyro[0], openimu_gyro[1], openimu_gyro[2],\
                            openimu_euler[0], openimu_euler[1], openimu_euler[2],\
                            imu381_euler[0], imu381_euler[1], imu381_euler[2])
            f.write(lines)
            f.flush()
            # 4. send over UDP
            packed_data = struct.pack('dddddddddd', openimu_euler[0], openimu_euler[1],\
                                        imu381_euler[0], imu381_euler[1],\
                                        0, 0,\
                                        0, 0, 0, 0)
            s.sendto(packed_data, (network, PORT))
    except KeyboardInterrupt:
        print("Stop logging, preparing data for simulation...")
        f.close()
        if openimu_unit['enable']:
            p_openimu.terminate()
            p_openimu.join()
        if imu381_unit['enable']:
            p_imu381.terminate()
            p_imu381.join()
        # post_proccess_for_ins_test.post_processing(data_file)
