import serial
import time

# serial config
port = 'com7'
baud = 230400

# log file
log_dir = './log_data/'
log_file = 'log.bin'

# open port
ser = serial.Serial(port, baud)
if ser.isOpen():
    print("Open %s"% port)
else:
    print("Fail to open %s"% port)
    exit()
# open log file
data_file = log_dir + log_file
f = open(data_file, 'wb+')
f.truncate()

# reset unit
print('Reset unit.')
reset_cmd='55555352007E4F'
ser.write(bytearray.fromhex(reset_cmd))
ser.reset_input_buffer()

# get serail data and write into the log file
try:
    print("Start logging.")
    while True:
        if ser.in_waiting:
            # print(ser.in_waiting)
            data = ser.read(ser.in_waiting)
            f.write(data)
        else:
            time.sleep(0.001)
            
except KeyboardInterrupt:
    print('End logging')
    ser.close()
    f.close()
