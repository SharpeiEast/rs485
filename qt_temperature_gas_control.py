#!/usr/bin/python
# -*- coding: UTF-8 -*-


import time
from datetime import datetime
import sys
import serial
import pandas as pd
import matplotlib.pyplot as plt
import os
from qt_http_sender import CGI_CMD
from RelayController import relayClose, relayOpen
from powerSupplyController import ser_turnOn


class Tempera_control():
    def __init__(self, ceiling,floor):
        self.ceiling = ceiling
        self.floor = floor

    def qt_gas_control(self, temp, t_banchmark, gas_status):
        host = "192.168.1.210"
        port = 8089
        if temp < t_banchmark and gas_status == 0:
            relayOpen(host, port, "1")
            gas_status = 1
            time.sleep(1)
        elif temp > t_banchmark and gas_status:
            relayClose(host, port, "1")
            gas_status = 0
            time.sleep(1)
        elif temp > self.ceiling and gas_status == 0:
            relayOpen(host, port, "1")
            gas_status = 1
            time.sleep(1)
        return gas_status

    def target_temp(self, temp, temperas):
        # temperas = [0,10,30,50,70,90,110]
        if temp > temperas[0]:
            temperas.pop(0)
            return temperas,min(temp + 20,self.ceiling)
        else:
            return temperas, False

    # def




if __name__ == '__main__':
    # host = "192.168.1.210"
    # port = 8089
    #
    # relayClose(host, port, "1")
    # time.sleep(1)
    # relayOpen(host, port, "1")
    # time.sleep(1)
    TC = Tempera_control(-40, 85)
    temp_set = TC.target_temp(110,[0,10,30,50,70,90,110])
    print(temp_set)

    # gas_sta = TC.qt_gas_control(-10, 30, 0)

# com = '/dev/ttyUSB0'
# cmd_text = 'echo "123456" | sudo -S sudo chmod 777 {}'.format(com)
# os.popen(cmd_text).read()
