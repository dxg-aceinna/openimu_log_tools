import math
import serial
import serial.tools.list_ports
from multiprocessing import Process, Pipe, Array
import time
import struct
import numpy as np
import attitude
import imu38x

#### INS381
ins381_unit = {'port':'COM7',\
            'baud':230400,\
            'packet_type':'a2',\
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
    #### find ports
    if not ins381_unit['enable']:
        ins381_unit['port'] = None
    print('%s is an OpenIMU.' % ins381_unit['port'])

    #### create pipes
    # ins381
    if ins381_unit['enable']:
        print('connecting to the unti with NXP accel...')
        parent_conn_nxp, child_conn_nxp = Pipe()
        process_target = log_imu38x
        p_ins381 = Process(target=process_target,\
                        args=(ins381_unit['port'], ins381_unit['baud'],\
                              ins381_unit['packet_type'], child_conn_nxp)
                       )
        p_ins381.daemon = True
        p_ins381.start()

    #### create log file
    data_file = log_dir + log_file
    f = open(data_file, 'w+')
    f.truncate()
    headerline = "recv_interval (s), openimu timer,"
    headerline += "ax (m/s2), ay (m/s2), az (m/s2),"
    headerline += "wx (deg/s), wy (deg/s), wz (deg/s),"
    headerline += "roll (deg), pitch (deg), yaw (deg)\n"
    f.write(headerline)
    f.flush()

    #### start logging
    # start time, to calculate recv interval
    tstart = time.time()
    # data from INS381
    ins381_timer = 0
    ins381_acc = np.zeros((3,))
    ins381_gyro = np.zeros((3,))
    ins381_euler = np.zeros((3,))
    # logging
    try:
        while True:
            # 1. timer interval
            tnow = time.time()
            time_interval = tnow-tstart
            tstart = tnow
            # 2. INS381, timer, acc and gyro, lla, vel, Euler angles
            if ins381_unit['enable']:
                latest_ins381 = parent_conn_nxp.recv()
                ins381_timer = latest_ins381[0]
                ins381_acc = np.array(latest_ins381[3])
                ins381_gyro = np.array(latest_ins381[2])
                ins381_euler = np.array(latest_ins381[1])
                if 'orientation' in ins381_unit:
                    ins381_acc = orientation(ins381_acc, ins381_unit['orientation'])
                    ins381_gyro = orientation(ins381_gyro, ins381_unit['orientation'])
            # 3. log data to file
            fmt = "%f, %u, "                    # itow, packet timer
            fmt += "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, "   # ins381 acc and gyro
            fmt += "%f, %f, %f\n" # ins381 lla/vel/euler
            lines = fmt% (\
                            time_interval, ins381_timer,\
                            ins381_acc[0], ins381_acc[1], ins381_acc[2],\
                            ins381_gyro[0], ins381_gyro[1], ins381_gyro[2],\
                            ins381_euler[0], ins381_euler[1], ins381_euler[2])
            f.write(lines)
            f.flush()
    except KeyboardInterrupt:
        print("Stop logging, preparing data for simulation...")
        f.close()
        if ins381_unit['enable']:
            p_ins381.terminate()
            p_ins381.join()
        # post_proccess_for_ins_test.post_processing(data_file)
