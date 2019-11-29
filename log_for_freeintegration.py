import math
import serial
import serial.tools.list_ports
from multiprocessing import Process, Pipe, Array
import time
import struct
import numpy as np
import attitude
import imu38x
import post_proccess_for_free_integration

units = [
            {
                'name':'bad',\
                'port':'COM7',\
                'baud':115200,\
                'packet_type':'S1',\
                'unit_type':'imu38x',\
                # 'orientation':'-y+x+z',\
                'enable':True
            },
            {
                'name':'good',\
                'port':'COM30',\
                'baud':115200,\
                'packet_type':'S1',\
                'unit_type':'imu38x',\
                # 'orientation':'-y+x+z',\
                'enable':True
            }
        ]

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
    #### connect to units
    enabled_units = []
    num_units = 0
    for i in units:
        if i['enable']:
            enabled_units.append(i)
            num_units += 1
            ### create pipes
            process_target = None
            if i['unit_type'].lower() == 'imu38x':
                process_target = log_imu38x
            i['pipe'] = Pipe()
            i['process'] = Process( target=process_target,\
                                    args=(i['port'], i['baud'],\
                                    i['packet_type'], i['pipe'][1])
                                  )
            i['process'].daemon = True
            i['process'].start()
            print('Connected to ' + i['name'] + ' on ' + i['port'])
    #### start log
    # start time, to calculate recv interval
    tstart = time.time()
    cntr = np.zeros((3,))
    acc = np.zeros((3*num_units,))
    gyro = np.zeros((3*num_units,))
    try:
        while num_units:
            # 1. timer interval
            tnow = time.time()
            time_interval = tnow-tstart
            tstart = tnow
            # 2. get latest data
            for i in range(num_units):
                latest = None
                if i == 0:
                    latest = enabled_units[i]['pipe'][0].recv()
                else:
                    while enabled_units[i]['pipe'][0].poll():
                        latest = enabled_units[i]['pipe'][0].recv()
                if latest is not None:
                    cntr[i] = latest[0]
                    acc[i*3:(i+1)*3] = latest[1]
                    gyro[i*3:(i+1)*3] = latest[2]
            # 3. log data to file
            fmt = "%f, %u, "                    # itow, packet timer
            fmt += "%f, %f, %f, %f, %f, %f, "   # 1st unit's acc and gyro
            fmt += "%f, %f, %f, %f, %f, %f, "   # 2nd unit's acc and gyro
            fmt += "%f, %f, %f, %f, %f, %f, "   # lla/vel
            fmt += "%f, %f, %f\n"               # Euler angles.
            lines = fmt% (\
                            time_interval, cntr[0],\
                            acc[0], acc[1], acc[2],\
                            gyro[0], gyro[1], gyro[2],\
                            acc[3], acc[4], acc[5],\
                            gyro[3], gyro[4], gyro[5],\
                            0, 0, 0, 0, 0, 0,\
                            0, 0, 0
                            )
            f.write(lines)
            f.flush()
    except KeyboardInterrupt:
        print("Stop logging, preparing data for simulation...")
        f.close()
        for i in enabled_units:
            i['process'].terminate()
            i['process'].join()
        post_proccess_for_free_integration.post_processing(data_file)
    