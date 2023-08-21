#!usr/bin/env python3
import serial
import struct

class IMUData:
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    leve = 0.0
    temp = 0.0

ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=1)
Re_buf = bytearray(30)
counter = 0
sign = 0
start_reg = 0
len = 1
add = 0xa4
imu = IMUData

while True:
    i = 0
    sum = 0
    com_input = ser.readline(1)
    if com_input:
        Re_buf.insert(counter,com_input[0])
        if counter == 0:
            if Re_buf[0] != add:
                continue
        elif counter == 1:
            if Re_buf[1] != 0x03:
                counter
                continue
        elif counter == 2:
            if Re_buf[2] < 0x2c:
                start_reg = Re_buf[2]
            else:
                counter = 0
                continue
        elif counter == 3:
            if start_reg + Re_buf[3] < 0x2c:
                len = Re_buf[3]
            else:
                counter = 0
                continue
        else:
            if len+5 == counter:
                sign = 1
        if sign:
            sign = 0
            for i in range(counter -1):
                sum += Re_buf[i]
                
                if sum == Re_buf[i]:
                    imu = struct.unpack('<hhhbh', bytes(Re_buf[4:13
                    ]))
                    print("roll: {:.2f}, pitch: {:.2f}, yaw: {:.2f}, temp: {:.2f}, leve: {:.2f}".format(
                            imu[0] / 100.0,
                            imu[1] / 100.0,
                            imu[2] / 100.0,
                            imu[4] / 100.0,
                            imu[3]
                            ))
                    counter = 0

        else:
            counter +=1

ser.close()



