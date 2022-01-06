'''
@Author: Weisong Wang
@Date: 2020-03-13 10:54:36
@description: 继电器控制脚本
@LastEditors: Weisong Wang
@LastEditTime: 2020-06-05 10:04:04
'''
# -*- coding: utf-8 -*-
import socket
import struct
import time
# from WebUpgrade import Log

ch = {"1": "00", "2": "01", "3": "02", "4": "03", "5": "04", "6": "05", "7": "06", "8": "07", "open_all": "10",
      "close_all": "11", "all_status": "12"}
ch_open = "FF"
ch_close = "00"
# logs = Log.SetLog("RelayControl")


def relayControl(host, port, channel, control_str):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)
    # s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, socket.IP_TTL)
    s.connect((host, port))
    # print("connect %s success" % host)
    if channel == "all_status":
        relay_ch = "12"
        control = "00"
        data = struct.pack('8B', int('AA', 16), int('5A', 16), int('00', 16), int(relay_ch, 16), int(control, 16),
                           int('00', 16), int('00', 16), int('FF', 16))
        # print(data)
        s.send(data)
        time.sleep(1)
        status = s.recv(1024).decode()
        print(status)
        s.close()
        return status
    elif channel == "open_all":
        relay_ch = "10"
        control = "00"
        data = struct.pack('8B', int('AA', 16), int('5A', 16), int('00', 16), int(relay_ch, 16), int(control, 16),
                           int('00', 16), int('00', 16), int('FF', 16))
        # print(data)
        s.send(data)
        time.sleep(1)
    elif channel == "close_all":
        relay_ch = "11"
        control = "00"
        data = struct.pack('8B', int('AA', 16), int('5A', 16), int('00', 16), int(relay_ch, 16), int(control, 16),
                           int('00', 16), int('00', 16), int('FF', 16))
        # print(data)
        s.send(data)
        time.sleep(1)
    else:
        relay_ch = channel
        control = control_str
        data = struct.pack('8B', int('AA', 16), int('5A', 16), int('00', 16), int(relay_ch, 16), int(control, 16),
                           int('00', 16), int('00', 16), int('FF', 16))
        # print(data)
        s.send(data)
        time.sleep(1)
    s.close()
    # s.shutdown(1)

def relayClose(host, port, channel):
    relay_control = ch_close
    if channel.upper() == "ALL":
        relay_ch = "close_all"
    elif int(channel) in range(0,9):
        relay_ch = ch[str(channel)]
    else:
        relay_ch = "all_status"
    relayControl(host, port, relay_ch, relay_control)
    # logs.info("close %s success" % channel)
    print("close %s success" % channel)


def relayOpen(host, port, channel):
    relay_control = ch_open
    if channel.upper() == "ALL":
        relay_ch = "open_all"
    elif int(channel) in range(0, 9):
        relay_ch = ch[str(channel)]
    else:
        relay_ch = "all_status"
    relayControl(host, port, relay_ch, relay_control)
    # logs.info("open %s success" % channel)
    print("open %s success" % channel)


def relayStatus(host, port):
    relay_ch = "all_status"
    relay_control = ch_close
    relayControl(host, port, relay_ch, relay_control)
    # logs.info("check all status")


if __name__ == '__main__':
    host = "192.168.1.210"
    port = 8089

    relayClose(host, port, "1")
    time.sleep(1)
    # relayOpen(host, port, "1")
    # time.sleep(1)
    # relayClose(host, port, "2")
    # time.sleep(1)
    # relayOpen(host, port, "2")
    # time.sleep(1)
    # relayClose(host, port, "3")
    # time.sleep(1)
    # relayOpen(host, port, "3")


