



import os
import time
from refrigerator_communication import Communication,dynamic_figure
from foundation_class import Lydar
from qt_temperature_gas_control import Tempera_control


# class Tempera_control():
#     def __init__(self, ceiling,floor):
#         self.ceiling = ceiling
#         self.floor = floor
#
#     def gas_control(self, temp, t_banchmark, gas_status):
#         host = "192.168.1.210"
#         port = 8089
#         if temp < t_banchmark and gas_status == 0:
#             relayOpen(host, port, "1")
#             gas_status = 1
#             time.sleep(1)
#         elif temp > t_banchmark and gas_status == 1:
#             relayClose(host, port, "1")
#             gas_status = 0
#             time.sleep(1)
#         elif temp > self.ceiling and gas_status == 0:
#             relayOpen(host, port, "1")
#             gas_status = 2
#             time.sleep(1)
#         return gas_status
#
#     def target_temp(self, temp, temperas):
#         # temperas = [0,10,30,50,70,90,110]
#         if len(temperas) != 0:
#             if temp > temperas[0]:
#                 last_temp = temperas.pop(0)
#                 return temperas, min(last_temp + 30.1, self.ceiling)
#             else:
#                 return temperas, False
#         else:
#             if temp > self.ceiling:
#                 return temperas, 0.1
#             else:
#                 return temperas, False


if __name__ == '__main__':
    temperas = [-30, -15, 0, 15, 30, 45, 60, 75, 90]  # AT升温范围
    TC = Tempera_control(-40, 110)  # AT安全温度上下限
    gas_sta = 0  # 吹气状态，0表示关闭，1表示开启
    tempera_index = 'work_temp'  #
    refrigerator_port = '/dev/ttyUSB0'
    # powerSupply_port = '/dev/ttyUSB0'
    cmd_text1 = 'echo "123456" | sudo -S sudo chmod 777 {}'.format(refrigerator_port)
    # cmd_text2 = 'echo "123456" | sudo -S sudo chmod 777 {}'.format(powerSupply_port)
    os.popen(cmd_text1).read()
    # os.popen(cmd_text2).read()

    file_name = time.strftime("%Y-%m-%d~%H-%M-%S", time.localtime()) + '.txt'
    print('按下回车开始计时，按下 Ctrl + C 停止计时。')
    Engine1 = Communication(refrigerator_port, 19200, 0.5)
    Engine1.Operation_options(1)
    Engine1.set_temperature(-70)
    # voltage = 24
    # ser_turnOn(float(voltage), sport=powerSupply_port)
    Lidar = Lydar()
    Lidar.__get_binfo_from_udp__()
    dict_inf = Lidar.get_statistic_cgi()
    d_figure = dynamic_figure(80,-50,120)
    Lidar_cgi_break = 0
    try:
        Lidar_tempera = float(dict_inf[tempera_index])
        print('雷达温度：', Lidar_tempera)
    except:
        print('胖子，你没接雷达!')

    while True:
        input("")  # 如果是 python 2.x 版本请使用 raw_input()
        starttime = time.time()
        # t_list = [30,31,32]
        print('开始')
        try:
            while True:
                print('-' * 20)
                print('计时: ', round(time.time() - starttime, 0), '秒')  # , end="\r")
                f = open(file_name, 'a')
                f.write('time:' + str(round( time.time()  - starttime, 1) )+ '秒')
               # f.write('\r\n')
               #  f.write('\n')
               #  f.close()
               # t_list, judge,rx_temp = temp_judge(t_list)
                current_tempera_1 = Engine1.check_temperature(1)
                current_tempera_2 = Engine1.check_temperature(2)
                current_tempera_set = Engine1.check_temperature(3)
                print('温槽温度采集完毕')
                # Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body']['TempRX'])
                try:
                    Lidar_tempera = float(Lidar.get_statistic_cgi()[tempera_index])
                except:
                    Lidar_cgi_break += 1
                    print('雷达掉线 + 1')
                gas_sta = TC.gas_control(Lidar_tempera, 50, gas_sta)
                temperas, temp_target = TC.target_temp(Lidar_tempera, temperas, 30.1)
                if temp_target != False:
                # if isinstance(temp_target, float):
                    print('set tempera as: ', temp_target)
                    Engine1.set_temperature(temp_target)
                # print('gas-sta: ', gas_sta)
                # print('tempera_target: ', temp_target)

                # if current_tempera_1 > 15 and current_tempera_set > -69 and Lidar_tempera > 10:
                #     Engine1.set_temperature(-70)
                # elif current_tempera_1 < -30 and current_tempera_set < 19 and Lidar_tempera < -10:
                #     Engine1.set_temperature(20)
                print('RS485 Temp is ' + str(round(current_tempera_1,2)) + ' ℃')
                print('Envir temp is ' + str(round(current_tempera_2,2)) + ' ℃')
                print('Target temp is ' + str(current_tempera_set) + ' ℃')
                print('Lidar temp is ' + str(Lidar_tempera) + ' ℃')
                d_figure.update(round(time.time() - starttime, 0), current_tempera_1, current_tempera_2,Lidar_tempera,current_tempera_set)
                print('*'*20)
              #  print('t_list: ',t_list)
              #  print(judge)
                # f = open('record_tempera.txt', 'a')
                f.write(' RSTemp: ' + str(current_tempera_1))
                f.write(' ENTemp: ' + str(current_tempera_2))
                f.write(' Target: ' + str(current_tempera_set))
                f.write(' Lidar_T: ' + str(Lidar_tempera))
                f.write('\n')
                # Lidar_tempera = 60
                # if judge:
                #     f.write('  Record Point Cloud')
                #     f.write('\r\n')
                f.close()
                time.sleep(2)
        except KeyboardInterrupt:
            print('结束')
            endtime = time.time()
            print('总共的时间为:', round(endtime - starttime, 2), 'secs')
            print('Lidar cgi break times: ', Lidar_cgi_break)
            break