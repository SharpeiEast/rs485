#!/usr/bin/python
# -*- coding: UTF-8 -*-



import time
from datetime import datetime
import sys
import serial
import pandas as pd
import matplotlib.pyplot as plt
import os

from RelayController import relayClose, relayOpen
from powerSupplyController import ser_turnOn

class QT_test_control():
    def __init__(self,l_host):
        self.host = l_host


com = '/dev/ttyUSB0'
cmd_text = 'echo "123456" | sudo -S sudo chmod 777 {}'.format(com)
os.popen(cmd_text).read()