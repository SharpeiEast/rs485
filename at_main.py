



import os
import time
from refrigerator_communication import Communication,dynamic_figure
from foundation_class import Lydar

class Tempera_control():
    def __init__(self, ceiling,floor):
        self.ceiling = ceiling
        self.floor = floor

    def gas_control(self, temp, t_banchmark, gas_status):
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
        if len(temperas) != 0:
            if temp > temperas[0]:
                last_temp = temperas.pop(0)
                return temperas, min(last_temp + 20.1, self.ceiling)
            else:
                return temperas, False
        else:
            if temp > self.ceiling:
                return 0.1, True
            else:
                return temperas, False


if __name__ == '__main__':

    temperas = [0, 10, 30, 50, 70, 90, 110] # AT升温范围
    TC = Tempera_control(-40, 110)
    gas_sta = 0

    file_name = time.strftime("%Y-%m-%d~%H-%M-%S", time.localtime()) + '.txt'
    print('按下回车开始计时，按下 Ctrl + C 停止计时。')
    Engine1 = Communication('/dev/ttyUSB0', 19200, 0.5)
    Lidar = Lydar()
    Lidar.__get_binfo_from_udp__()
    dict_inf = Lidar.get_statistic_cgi()

    d_figure = dynamic_figure(80,-50,120)
    Lidar_cgi_break = 0
    try:
        Lidar_tempera = float(Lidar.dict_inf['work_temp'])
        print('雷达温度：', Lidar_tempera)
    except:
        print('胖子，你没接雷达')

    while True:
        input("")  # 如果是 python 2.x 版本请使用 raw_input()
        starttime = time.time()
        # t_list = [30,31,32]
        print('开始')
        try:
            while True:
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
                # Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body']['TempRX'])
                try:
                    Lidar_tempera = float(Lidar.dict_inf['work_temp'])
                except:
                    Lidar_cgi_break += 1
                gas_sta = TC.gas_control(Lidar_tempera, 50, gas_sta)
                tempera_target, temp_status = TC.target_temp(Lidar_tempera, temperas)
                if temp_status:
                    Engine1.set_temperature(tempera_target)
                print('gas-sta: ', gas_sta)
                print('tempera_target: ', tempera_target)

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
                print('-'*20)
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