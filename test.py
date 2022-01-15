
#! /usr/bin/python
#-*- coding: utf-8 -*-

import configparser

path = 'lidar_xma.ini'

cf=configparser.ConfigParser()

cf.read(path) #读配置文件（ini、conf）返回结果是列表

cf.sections() #获取读到的所有sections(域)，返回列表类型

cf.options('test') #某个域下的所有key，返回列表类型

key_value = cf.items('baseconf') #某个域下的所有key，value对
print(type(key_value[1][1]))

value=cf.get('test','ip')#获取某个yu下的key对应的value值

print(value)
print(type(value))

# cf.type(value) #获取的value值的类型

# import os
# import time
# from refrigerator_communication import Communication,dynamic_figure
# from qt_http_sender import CGI_CMD
# from qt_temperature_gas_control import Tempera_control
# from powerSupplyController import ser_turnOn
# from foundation_class import Lydar
#
# if __name__ == '__main__':
#
#     temperas = [0, 10, 30, 50, 70, 85] # QT升温范围
#     TC = Tempera_control(-40,85)
#     gas_sta = 0
#     tempera_index = 'TempRX'
#     refrigerator_port = '/dev/ttyUSB0'
#     # powerSupply_port = '/dev/ttyUSB1'
#     cmd_text1 = 'echo "123456" | sudo -S sudo chmod 777 {}'.format(refrigerator_port)
#     # cmd_text2 = 'echo "123456" | sudo -S sudo chmod 777 {}'.format(powerSupply_port)
#     os.popen(cmd_text1).read()
#     # os.popen(cmd_text2).read()
#
#     file_name = time.strftime("%Y-%m-%d~%H-%M-%S", time.localtime()) + '.txt'
#     print('按下回车开始计时，按下 Ctrl + C 停止计时。')
#     Engine1 = Communication(refrigerator_port, 19200, 0.5)
#     Engine1.Operation_options(0)
#     Engine1.set_temperature(-50)
#     Lidar = Lydar()
#     Lidar.__get_binfo_from_udp__()
#     d_figure = dynamic_figure(80, -50, 120)
#     Lidar_cgi_break = 0
#
#     try:
#         Lidar_tempera = Lidar.parse_reserved_bits_onerow()['TempA_RFB_1']
#         print('雷达温度：', Lidar_tempera)
#     except:
#         print('胖子，你没接雷达')

    # voltage = 24
    # ser_turnOn(float(voltage), sport=powerSupply_port)
    #
    # current_tempera_1 = Engine1.check_temperature(1)
    # current_tempera_2 = Engine1.check_temperature(2)
    # current_tempera_set = Engine1.check_temperature(3)
    # print(current_tempera_1, current_tempera_2, current_tempera_set)
    # Lidar_tempera = 1
    # gas_sta = TC.gas_control(Lidar_tempera, 50, gas_sta)
    # print(gas_sta)
    # temperas, temp_target = TC.target_temp(Lidar_tempera, temperas)
    # print(temperas, temp_target)
    # if temp_target != False:
    #     Engine1.set_temperature(temp_target)
    # print('RS485 Temp is ' + str(round(current_tempera_1, 2)) + ' ℃')
    # print('Envir temp is ' + str(round(current_tempera_2, 2)) + ' ℃')
    # print('Target temp is ' + str(current_tempera_set) + ' ℃')
    # print('Lidar temp is ' + str(Lidar_tempera) + ' ℃')
    # Engine1.set_temperature(-20.1)

    # tempera_index = 'TempRX'  # 如果是B样雷达则更改为'TempAvrg'
    # Lidar = CGI_CMD(host='192.168.1.201')
    # Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body'][tempera_index])
    # print(Lidar_tempera)

