import os
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import attitude
import kml.dynamic_kml as kml


#### prepare data for free integration simulation
data_dir = "./ins_data/"


def post_processing(data_file):
    #### create data dir
    if not os.path.exists(data_dir):
        try:
            os.makedirs(data_dir)
        except:
            raise IOError('Cannot create dir: %s.'% data_dir)
    #### read logged file
    data = np.genfromtxt(data_file, delimiter=',', skip_header=1)
    # remove zero LLA/Vel/att from ins1000
    data = data[100:, :]
    lla = data[:, 8:11]
    vel = data[:, 11:14]
    euler = data[:, 14:17]
    ref_lla = data[:, 17:20]
    ref_vel = data[:, 20:23]
    ref_euler = data[:, 23:26]
    # Generate logged files
    file_name = data_dir + "pos-0.csv"
    headerline = "pos_lat (deg),pos_lon (deg),pos_alt (m)"
    np.savetxt(file_name, lla, header=headerline, delimiter=',', comments='')
    file_name = data_dir + "ref_pos.csv"
    headerline = "ref_pos_lat (deg),ref_pos_lon (deg),ref_pos_alt (m)"
    np.savetxt(file_name, ref_lla, header=headerline, delimiter=',', comments='')
    file_name = data_dir + "att_euler-0.csv"
    headerline = "Yaw (deg),Pitch (deg),Roll (deg)"
    np.savetxt(file_name, euler[:,-1:-4:-1], header=headerline, delimiter=',', comments='')
    # generate .kml file
    file_name = data_dir + 'ref_pos.kml'
    kml.gen_kml(file_name, ref_lla, ref_euler[:,2], 'ff0000ff')
    file_name = data_dir + 'pos-1.kml'
    kml.gen_kml(file_name, lla, euler[:,2], 'ff00ff00')
    

if __name__ == "__main__":
    # data_file = "E:\\Projects\\python-imu380-mult\\log_data\\log-5.csv"
    data_file = "E:\\vs_projects\\dmu380_offline_sim-ins\\sim_app\\x64\\Debug\\results.csv"
    post_processing(data_file)