import os
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import attitude


#### prepare data for free integration simulation
data_dir = "./free_integration_data/"
dt = 0.01
# only log 10s data for simulation. Data for initialization will be discared anyway.
# If limit_data_to_10s is True, only 10sec data after initialization will be logged.
limit_data_to_10s = True
# force initial velocity to be zero, otherwise averaged INS1000 output will be used.
zero_ini_vel = True
# using averaged accelerometer output to get initial pitch and roll,
#   otherwiese averaged INS1000 output will be used.
acc_ini_att = True

def post_processing(data_file, nav_view=False):
    #### create data dir
    if not os.path.exists(data_dir):
        try:
            os.makedirs(data_dir)
        except:
            raise IOError('Cannot create dir: %s.'% data_dir)
    #### read logged file
    if nav_view:
        data = np.genfromtxt(data_file, delimiter='\t', skip_header=15)
        acc0 = data[:, 1:4] * 9.80665
        gyro0 = data[:, 4:7]
        lla = np.zeros((acc0.shape[0], 3))
        vel = np.zeros((acc0.shape[0], 3))
        euler = np.zeros((acc0.shape[0], 3))
    else:
        data = np.genfromtxt(data_file, delimiter=',', skip_header=1)
        # remove zero LLA/Vel/att from ins1000
        data = data[100:, :]
        acc0 = data[:, 2:5]
        gyro0 = data[:, 5:8]
        acc1 = data[:, 8:11]
        gyro1 = data[:, 11:14]
        lla = data[:, 14:17]
        vel = data[:, 17:20]
        euler = data[:, 22:19:-1]
    '''
    Generate logged files.
    You can specify multiple start points to generate multiple sets of data for simulaiton. 
    '''
    # get data before motion to calculate initial states
    plt.ion()
    plt.plot(acc0)
    plt.grid(True)
    plt.pause(0.01)
    # plt.show(block=False)
    idx_str = input('Please input start index of the motion: ')
    # idx = parse_index(idx_str)
    # index should be a positive integer
    idx0 = int(idx_str)
    if idx0 < 1:
        idx0 = 1

    # limit logged data to 10s
    if limit_data_to_10s:
        n = idx0 + int(10.0/dt)
        if n > gyro0.shape[0]:
            n = gyro0.shape[0]
    else:
        n = gyro0.shape[0]

    # generate initial states files
    # generate reference data files
    gen_ref_data_files(lla, vel, euler, idx0, n, data_dir)
    # generate initial states and sensor files
    gen_sensor_files(gyro0, acc0, lla, vel, euler, idx0, n, data_dir, key=0)
    if not nav_view:
        gen_sensor_files(gyro1, acc1, lla, vel, euler, idx0, n, data_dir, key=1)
        # combine two ini files into one, and delete unused files
        ini_0 = np.genfromtxt(data_dir + "ini-0.txt", delimiter=',')    # row vector
        ini_1 = np.genfromtxt(data_dir + "ini-1.txt", delimiter=',')    # row vector
        ini_states = np.vstack([ini_0, ini_1])
        np.savetxt(data_dir + "ini.txt", ini_states.T, delimiter=',', comments='')
        os.remove(data_dir + "ini-0.txt")
        os.remove(data_dir + "ini-1.txt")        

def gen_ref_data_files(lla, vel, euler, idx0, n, dir):
    # create dir if it does not exist
    if not os.path.exists(dir):
        os.mkdir(dir)
    # time
    time = np.array(range(0, n-idx0)) * dt
    file_name = dir + "time.csv"
    headerline = "time (sec)"
    np.savetxt(file_name, time, header=headerline, delimiter=',', comments='')
    # ref_pos
    file_name = dir + "ref_pos.csv"
    headerline = "ref_pos_lat (deg),ref_pos_lon (deg),ref_pos_alt (m)"
    np.savetxt(file_name, lla[idx0:n, :], header=headerline, delimiter=',', comments='')
    # ref_vel
    file_name = dir + "ref_vel.csv"
    headerline = "ref_vel_x (m/s),ref_vel_y (m/s),ref_vel_z (m/s)"
    np.savetxt(file_name, vel[idx0:n, :], header=headerline, delimiter=',', comments='')
    # ref_att_euler
    file_name = dir + "ref_att_euler.csv"
    headerline = "ref_Yaw (deg),ref_Pitch (deg),ref_Roll (deg)"
    np.savetxt(file_name, euler[idx0:n, :], header=headerline, delimiter=',', comments='')

def gen_sensor_files(gyro, acc, lla, vel, euler, idx0, n, dir, key=0):
    # create dir if it does not exist
    if not os.path.exists(dir):
        os.mkdir(dir)
    key_str = str(key)
    # gyro bias
    wb = np.average(gyro[0:idx0,:], axis=0)
    # accel bias, not used
    ab = np.average(acc[0:idx0,:], axis=0)
    ab_norm = math.sqrt(np.dot(ab, ab))
    # initial pos
    ini_pos = np.average(lla[0:idx0,:], axis=0)
    # initial attitude
    ini_euler = np.average(euler[0:idx0,:], axis=0)
    unit_gravity = -1.0 * ab / ab_norm
    if acc_ini_att:
        ini_euler[1] = -math.asin(unit_gravity[0]) * attitude.R2D
        ini_euler[2] = math.atan2(unit_gravity[1], unit_gravity[2]) * attitude.R2D
    # initial vel
    if zero_ini_vel:
        ini_vel = np.zeros((3,))
    else: 
        ini_vel = np.average(vel[0:idx0,:], axis=0)
    # all initial states
    ini_states = np.hstack((ini_pos, ini_vel, ini_euler, ab_norm))
    #### create log file
    # ini states
    file_name = dir + "ini-" + key_str + ".txt"
    np.savetxt(file_name, ini_states, delimiter=',', comments='')
    # acc
    file_name = dir + "accel-" + key_str + ".csv"
    headerline = "accel_x (m/s^2),accel_y (m/s^2),accel_z (m/s^2)"
    np.savetxt(file_name, acc[idx0:n, :], header=headerline, delimiter=',', comments='')
    # gyro
    file_name = dir + "gyro-" + key_str + ".csv"
    headerline = "gyro_x (deg/s),gyro_y (deg/s),gyro_z (deg/s)"
    np.savetxt(file_name, gyro[idx0:n, :]-wb, header=headerline, delimiter=',', comments='')

def parse_index(idx):
    ii = idx.find(':')
    if ii >= 0:
        # start
        if ii == 0:
            start = 0
        else:
            start = int(idx[0:ii])
        # step
        idx = idx[ii+1:]
        ii = idx.find(':')
        if ii >= 0:
            if ii == 0:
                step = 1
            else:
                step = int(idx[0:ii])
            stop = int(idx[ii+1:])
        else:
            step = 1
            stop = int(idx)
        return list(range(start, stop+1, step))
    else:
        return [int(idx)]

if __name__ == "__main__":
    data_file = "E:\\Projects\\python-imu380-mult\\log_data\\log.csv"
    post_processing(data_file, nav_view=False)