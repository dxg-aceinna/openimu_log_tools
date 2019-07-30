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
ins381_unit = {'port':'COM7',\
            'baud':230400,\
            'packet_type':'id',\
            'unit_type':'imu38x',\
            # 'orientation':'-y+x+z',\
            'enable':True}
#### INS1000
ins1000_unit = {'port':'COM15',\
                'baud':230400,\
                'packet_type':'nav',\
                'unit_type':'ins1000',\
                'enable':False}

log_dir = './log_data/'
log_file = 'log.csv'

enable_kml = True

def log_imu38x(port, baud, packet, pipe):
    imu38x_unit = imu38x.imu38x(port, baud, packet_type=packet, pipe=pipe)
    imu38x_unit.start(reset=True)

def log_ins1000(port, baud, pipe):
    ins = ins1000.ins1000(port, baud, pipe)
    ins.start()


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
    if not ins1000_unit['enable']:
        ins1000_unit['port'] = None
    print('%s is an INS381.' % ins381_unit['port'])
    print('%s is an INS1000.' % ins1000_unit['port'])

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

    # ins1000
    if ins1000_unit['enable']:
        print('connecting to INS1000...')
        parent_conn_ins1000, child_conn_ins1000 = Pipe()
        p_ins1000 = Process(target=log_ins1000,\
                            args=(ins1000_unit['port'], ins1000_unit['baud'],\
                                  child_conn_ins1000)
                           )
        p_ins1000.daemon = True
        p_ins1000.start()

    #### create log file
    data_file = log_dir + log_file
    f = open(data_file, 'w+')
    f.truncate()
    headerline = "recv_interval (s), openimu timer,"
    headerline += "ax (g), ay (g), az (g),"
    headerline += "wx (deg/s), wy (deg/s), wz (deg/s),"
    headerline += "Lat (deg), Lon (deg), Alt (m),"
    headerline += "vN (m/s), vE (m/s), vD (m/s),"
    headerline += "roll (deg), pitch (deg), yaw (deg),"
    headerline += "ref_Lat (deg), ref_Lon (deg), ref_Alt (m),"
    headerline += "ref_vN (m/s), ref_vE (m/s), ref_vD (m/s),"
    headerline += "ref_roll (deg), ref_pitch (deg), ref_yaw (deg),"
    headerline += "hdop, hAcc, vAcc, gps_update, gps_valid\n"
    f.write(headerline)
    f.flush()

    #### start logging
    # start time, to calculate recv interval
    tstart = time.time()
    # data from INS381
    ins381_timer = 0
    gps_itow = 0
    ins381_acc = np.zeros((3,))
    ins381_gyro = np.zeros((3,))
    ins381_lla = np.zeros((3,))
    ins381_vel = np.zeros((3,))
    ins381_euler = np.zeros((3,))
    # data from ins1000
    ref_lla = np.zeros((3,))
    ref_vel = np.zeros((3,))
    ref_euler = np.zeros((3,))
    ref_accuracy = np.zeros((3,))
    gps_update = 0
    gps_valid = 0
    # logging
    counter = 0
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
                gps_itow = latest_ins381[1]
                ins381_acc = np.array(latest_ins381[2])
                ins381_gyro = np.array(latest_ins381[3])
                ins381_lla = np.array(latest_ins381[4])
                ins381_vel = np.array(latest_ins381[5])
                ins381_euler = np.array(latest_ins381[6])
                if 'orientation' in ins381_unit:
                    ins381_acc = orientation(ins381_acc, ins381_unit['orientation'])
                    ins381_gyro = orientation(ins381_gyro, ins381_unit['orientation'])
            # 3. ins1000 time, lla, vel and quat
            if ins1000_unit['enable']:
                # ins1000 can be of higher sampling rate, get the latest one
                latest_ref = None
                while parent_conn_ins1000.poll():
                    latest_ref = parent_conn_ins1000.recv()
                if latest_ref is not None:
                    ref_lla = np.array(latest_ref[1])
                    ref_vel = np.array(latest_ref[2])
                    ref_quat = np.array(latest_ref[3])
                    try:
                        ref_euler = attitude.quat2euler(ref_quat)   #ypr
                    except:
                        print("quat: %s"% latest_ref[3])
                    ref_euler[0] = ref_euler[0] * attitude.R2D
                    ref_euler[1] = ref_euler[1] * attitude.R2D
                    ref_euler[2] = ref_euler[2] * attitude.R2D
            else:
                ref_lla = np.array(latest_ins381[7])
                ref_vel = np.array(latest_ins381[8])
                ref_euler[0] = latest_ins381[9]
                ref_accuracy = np.array(latest_ins381[10])
                gps_update = latest_ins381[11] & 0x01
                gps_valid = (latest_ins381[11] >>1 ) & 0x01

            # 5. log data to file
            fmt = "%u, %u, "                    # itow, packet timer
            fmt += "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, "   # ins381 acc and gyro
            fmt += "%.9f, %.9f, %f, %f, %f, %f, %f, %f, %f, " # ins381 lla/vel/euler
            fmt += "%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, " # ref lla/vel/euler
            fmt += "%.9f, %.9f, %.9f, "     # ref accuracy (hdop, horizontal/vertical accuracy)
            fmt += "%u, %u\n"               # gps_update, gps_valid
            lines = fmt% (\
                            gps_itow, ins381_timer,\
                            ins381_acc[0], ins381_acc[1], ins381_acc[2],\
                            ins381_gyro[0], ins381_gyro[1], ins381_gyro[2],\
                            ins381_lla[0], ins381_lla[1], ins381_lla[2],\
                            ins381_vel[0], ins381_vel[1], ins381_vel[2],\
                            ins381_euler[0], ins381_euler[1], ins381_euler[2],\
                            ref_lla[0], ref_lla[1], ref_lla[2],\
                            ref_vel[0], ref_vel[1], ref_vel[2],\
                            ref_euler[2], ref_euler[1], ref_euler[0],\
                            ref_accuracy[0], ref_accuracy[1], ref_accuracy[2],\
                            gps_update, gps_valid)
            f.write(lines)
            f.flush()
            counter += 1
            if enable_kml and counter == 10:
                counter = 0
                kml.gen_kml('./kml/ins381.kml', ins381_lla, ins381_euler[2], 'ffff0000')
                kml.gen_kml('./kml/ins1000.kml', ref_lla, ref_euler[0], 'ff0000ff')
    except KeyboardInterrupt:
        print("Stop logging, preparing data for simulation...")
        f.close()
        if ins381_unit['enable']:
            p_ins381.terminate()
            p_ins381.join()
        if ins1000_unit['enable']:
            p_ins1000.terminate()
            p_ins1000.join()
        post_proccess_for_ins_test.post_processing(data_file)
