#!/usr/bin/python
#coding:utf-8

# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

font = FontProperties(fname=r"/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")


def tempera_time(tempera_record, name, title, col1, col2):  # input parameter should with filename extension, eg: .txt
    tempera_step = np.loadtxt(tempera_record)

    plt.figure(figsize=(10, 8))
    plt.plot(tempera_step[:,0], tempera_step[:,col1])
    plt.plot(tempera_step[:,0], tempera_step[:,col2])
    plt.legend(['Liquid Temperature', 'Lidar Temperature'], loc='best', fontsize=20, shadow=True
                              , edgecolor='blue')
    plt.title(title, fontproperties=font,fontsize=15)
    plt.xlabel("Time (s)",fontsize=15)
    plt.ylabel("Temperature (℃)",fontsize=15)
    plt.savefig('Temperature curve' + name + ".png", bbox_inches='tight')
    plt.clf()
    return 1
def tempera_time_single(tempera_record, name, title, col):
    tempera_step = np.loadtxt(tempera_record)
    plt.figure(figsize=(10, 8))
    plt.plot(tempera_step[:, 0], tempera_step[:, col])
    plt.legend(['Lidar Temperature'], loc='best', fontsize=20, shadow=True
               , edgecolor='blue')
    plt.title(title, fontproperties=font,fontsize=15)
    plt.xlabel("Time (s)",fontsize=15)
    plt.ylabel("Temperature (℃)",fontsize=15)
    plt.savefig('Temperature curve' + name + ".png", bbox_inches='tight')
    plt.clf()
    return 1


QT_as_load = tempera_time('2021-12-28~19-00-07 (copy).txt', '_QT', 'Temperature curve with QT Lidar as load',1,4)
QT_raise = tempera_time('2021-12-29~14-29-36 (copy).txt', '_QT_R','Heating curve with QT Lidar as load',1,4)  #刹不住车
QT_circle_fail = tempera_time('2021-12-29~16-47-39 (copy).txt', '_QT_broken', '升温曲线',1,4) # broken
QT_circle = tempera_time('2021-12-29~17-55-41 (copy).txt', '_QT_circle', '雷达温度循环曲线',1,4)
QT_circle_ = tempera_time_single('2021-12-31~10-15-36_0011a.txt', '_QT_120', 'QT雷达温度曲线（冲击机降温）', 1)



# See PyCharm help at https://www.jetbrains.com/help/pycharm/
