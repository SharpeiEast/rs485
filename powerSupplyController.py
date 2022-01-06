#!/usr/bin/python
# -*- coding: UTF-8 -*-
"""
supply controller
@data:2020-08
@author: Jackey Pu
"""

import time
from datetime import datetime
import sys
import serial
import pandas as pd
import matplotlib.pyplot as plt
import os


def ser_turnOn(vol, sport='COM3'):
    ser = serial.Serial(port=sport, baudrate=9600, bytesize=8,
                        parity='N', stopbits=1, timeout=1)
    # ser.open()
    if not ser.is_open:
        ser.close()
        ser = serial.Serial(port=sport, baudrate=9600,
                            bytesize=8, parity='N', stopbits=1, timeout=1)
    vol = float(vol)
    # ser.write(("out0\n").encode())
    # track = 0
    # time.sleep(20)#modify by pu
    if vol <= 30:
        ser.write(("track2\n").encode())
        ser.write(("vset1:" + str(vol) + "\n").encode())

    elif vol <= 55:
        ser.write(("track1\n").encode())
        ser.write(("vset1:" + str(vol / 2) + "\n").encode())
        # ser.write(("vset2:" + str(vol/2) + "\n").encode())

    else:
        print('voltage too large')
        os._exit(0)

    # ser.write(("vset1:"+str(vol)+"\n").encode())
    time.sleep(1)
    ser.write(("out1\n").encode())
    time.sleep(1)
    ser.close()


def ser_turnOff(sport='COM3'):
    ser = serial.Serial(port=sport, baudrate=9600, bytesize=8,
                        parity='N', stopbits=1, timeout=1)
    # ser.open()
    if not ser.is_open:
        ser.close()
        ser = serial.Serial(port=sport, baudrate=9600,
                            bytesize=8, parity='N', stopbits=1, timeout=1)
    time.sleep(1)
    ser.write(("out0\n").encode())
    time.sleep(1)
    ser.close()


def ser_read_i(volt, sport='COM3'):
    ser = serial.Serial(port=sport, baudrate=9600, bytesize=8,
                        parity='N', stopbits=1, timeout=1)
    # ser.open()
    if not ser.is_open:
        ser.close()
        ser = serial.Serial(port=sport, baudrate=9600,
                            bytesize=8, parity='N', stopbits=1, timeout=1)

    ser.write(("iout1?\n").encode())
    s_i = ser.readline()
    s_i = bytes.decode(s_i).split("A")[0]
    s_i = float(s_i)
    ser.close()
    track = 0
    if volt <= 30:
        track = 2
    elif volt <= 55:
        track = 1
    if track == 2:
        return s_i * 2
    elif track == 1:
        return s_i


def ser_read_v(volt, sport='COM3'):
    ser = serial.Serial(port=sport, baudrate=9600, bytesize=8,
                        parity='N', stopbits=1, timeout=1)
    # ser.open()
    if not ser.is_open:
        ser.close()
        ser = serial.Serial(port=sport, baudrate=9600,
                            bytesize=8, parity='N', stopbits=1, timeout=1)

    ser.write(("vout1?\n").encode())
    s_v = ser.readline()
    s_v = bytes.decode(s_v).split("V")[0]
    s_v = float(s_v)
    ser.close()
    track = 0
    if volt <= 30:
        track = 2
    elif volt <= 55:
        track = 1
    if track == 2:
        return s_v
    elif track == 1:
        return s_v * 2


def power_calc(vol, s_i):
    serial_power_info = (round(vol, 3), round(s_i, 3), round((vol * s_i) - s_i * s_i * 0.42, 3))
    return serial_power_info


def power_monitor(v, com='COM3'):
    s_i = ser_read_i(v, com)
    s_v = ser_read_v(v, com)
    p = power_calc(s_v, s_i)
    return p


def powerTest_by_supply(com, times=100, v=12, sn='Lidar'):
    file = os.path.join('./', '%s_power_by_supply_%s_%s.csv' % (sn, v, datetime.now().strftime("%Y%m%d%H%M")))

    with open(file, 'w+') as f:
        f.write('current' + ',' + 'volt' + ',' + 'power' + '\n')

        ser_turnOn(v, com)
        # time.sleep(60)
        for i in range(0, times):
            time.sleep(0.5)
            s_i = ser_read_i(v, com)
            s_v = ser_read_v(v, com)
            p = power_calc(s_v, s_i)
            print("volt is %f current is %f " % (s_v, s_i))
            # print(p)
            f.write(str(s_i) + ',' + str(s_v) + ',' + str(p[2]) + '\n')
        ser_turnOff(com)
        time.sleep(2)


if __name__ == '__main__':
    # sn = input("Please input the Lidar sn!\n")
    # com = 'COM3'
    # times = 120
    # vlist = [12]
    # for v in vlist:
    #     powerTest_by_supply(com, times, v, sn)

    com = '/dev/ttyUSB0'
    cmd_text = 'echo "123456" | sudo -S sudo chmod 777 {}'.format(com)
    os.popen(cmd_text).read()

    voltage = 24
    power_port = '/dev/ttyUSB0'
    ser_turnOn(float(voltage), sport=power_port)
