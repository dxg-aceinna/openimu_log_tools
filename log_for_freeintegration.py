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
import post_proccess_for_free_integration
    
#### Unit with NXP accel
nxp_unit = {'port':'COM11',\
            'baud':115200,\
            'packet_type':'z1',\
            'unit_type':'openimu',\
            'enable':True}
#### Unit with Bosch accel
bosch_unit = {'port':'COM30',\
              'baud':115200,\
              'packet_type':'s0',\
              'unit_type':'imu38x',\
              'enable':True}
#### INS1000
ins1000_unit = {'port':'COM15',\
                'baud':230400,\
                'packet_type':'nav',\
                'unit_type':'ins1000',\
                'enable':False}

log_dir = './log_data/'
log_file = 'log.csv'

def log_openimu(port, baud, packet, pipe):
    imu = openimu.openimu(port, baud, pipe)
    imu.start()

def log_imu38x(port, baud, packet, pipe):
    new_unit = imu38x.imu38x(port, baud, packet_type=packet, pipe=pipe)
    new_unit.start()

def log_ins1000(port, baud, pipe):
    ins = ins1000.ins1000(port, baud, pipe)
    ins.start()

if __name__ == "__main__":
    #### find ports
    if not bosch_unit['enable']:
        bosch_unit['port'] = None
    if not ins1000_unit['enable']:
        ins1000_unit['port'] = None
    print('%s is an NXP accel.' % nxp_unit['port'])
    print('%s is an Bosch accel.'% bosch_unit['port'])
    print('%s is an INS1000.' % ins1000_unit['port'])

    #### create pipes
    # nxp, it is in an openimu
    if nxp_unit['enable']:
        print('connecting to the unti with NXP accel...')
        parent_conn_nxp, child_conn_nxp = Pipe()
        if nxp_unit['unit_type'] == 'imu38x':
            process_target = log_imu38x
        elif nxp_unit['unit_type'] == 'openimu':
            process_target = log_openimu
        p_nxp = Process(target=process_target,\
                        args=(nxp_unit['port'], nxp_unit['baud'],\
                              nxp_unit['packet_type'], child_conn_nxp)
                       )
        p_nxp.daemon = True
        p_nxp.start()
    # bosch, it is in imu38x
    if bosch_unit['enable']:
        print('connecting to the unti with Bosch accel...')
        parent_conn_bosch, child_conn_bosch = Pipe()
        p_bosch = Process(target=log_imu38x,\
                          args=(bosch_unit['port'], bosch_unit['baud'],\
                                bosch_unit['packet_type'], child_conn_bosch)
                         )
        p_bosch.daemon = True
        p_bosch.start()
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
    headerline += "ax0 (m/s2), ay0 (m/s2), az0 (m/s2),"
    headerline += "wx0 (deg/s), wy0 (deg/s), wz0 (deg/s),"
    headerline += "ax1 (m/s2), ay1 (m/s2), az1 (m/s2),"
    headerline += "wx1 (deg/s), wy1 (deg/s), wz1 (deg/s),"
    headerline += "Lat (deg), Lon (deg), Alt (m),"
    headerline += "vN (m/s), vE (m/s), vD (m/s),"
    headerline += "roll (deg), pitch (deg), yaw (deg)\n"
    f.write(headerline)
    f.flush()

    #### start logging
    # start time, to calculate recv interval
    tstart = time.time()
    # data from unit with nxp accel
    nxp_timer = 0
    nxp_acc = np.zeros((3,))
    nxp_gyro = np.zeros((3,))
    # data from unti with bosch accel
    bosch_acc = np.zeros((3,))
    bosch_gyro = np.zeros((3,))
    # data from ins1000
    ref_lla = np.zeros((3,))
    ref_vel = np.zeros((3,))
    latest_ref_euler = np.zeros((3,))
    # logging
    try:
        while True:
            # 1. timer interval
            tnow = time.time()
            time_interval = tnow-tstart
            tstart = tnow
            # 2. openimu timer, acc and gyro
            latest_nxp = parent_conn_nxp.recv()
            nxp_timer = latest_nxp[0]
            nxp_acc = latest_nxp[1]
            nxp_gyro = latest_nxp[2]
            # 3. bosch unit
            if( bosch_unit['enable']):
                latest_bosch = parent_conn_bosch.recv()
                bosch_acc = latest_bosch[1]
                bosch_gyro = latest_bosch[2]
            # 4. ins1000 time, lla, vel and quat
            if( ins1000_unit['enable'] and parent_conn_ins1000.poll()):
                latest_ref = parent_conn_ins1000.recv()
                ref_lla = latest_ref[1]
                ref_vel = latest_ref[2]
                try:
                    latest_ref_euler = attitude.quat2euler(latest_ref[3])   #ypr
                except:
                    print("quat: %s"% latest_ref[3])
                latest_ref_euler[0] = latest_ref_euler[0] * attitude.R2D
                latest_ref_euler[1] = latest_ref_euler[1] * attitude.R2D
                latest_ref_euler[2] = latest_ref_euler[2] * attitude.R2D
            # 5. log data to file
            fmt = "%f, %u, "                    # time_interval, packet timer
            fmt += "%f, %f, %f, %f, %f, %f, "   # nxp acc and gyro
            fmt += "%f, %f, %f, %f, %f, %f, "   # bosch acc and gyro
            fmt += "%.9f, %.9f, %.9f, %f, %f, %f, %f, %f, %f\n" # ref lla/vel/euler
            lines = fmt% (\
                            time_interval, nxp_timer,\
                            nxp_acc[0], nxp_acc[1], nxp_acc[2],\
                            nxp_gyro[0], nxp_gyro[1], nxp_gyro[2],\
                            bosch_acc[0], bosch_acc[1], bosch_acc[2],\
                            bosch_gyro[0], bosch_gyro[1], bosch_gyro[2],\
                            ref_lla[0], ref_lla[1], ref_lla[2],\
                            ref_vel[0], ref_vel[1], ref_vel[2],
                            latest_ref_euler[2], latest_ref_euler[1], latest_ref_euler[0])
            f.write(lines)
            f.flush()
    except KeyboardInterrupt:
        print("Stop logging, preparing data for simulation...")
        f.close()
        p_nxp.terminate()
        p_nxp.join()
        if bosch_unit['enable']:
            p_bosch.terminate()
            p_bosch.join()
        if ins1000_unit['enable']:
            p_ins1000.terminate()
            p_ins1000.join()
        post_proccess_for_free_integration.post_processing(data_file)
