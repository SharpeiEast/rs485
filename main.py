# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


import os
import time
from refrigerator_communication import Communication,dynamic_figure
from qt_http_sender import CGI_CMD

def temp_difference_cgi_qt(Tx, lidar):  # lidar应该是一个类，qt = CGI_CMD(host='192.168.1.201')，Tx一般是恒温槽温度
    RX_temp = lidar.cgi_get_factory_monitor()['Body']['TempRxAvrg'] #读取rx的平均温度
    return Tx - float(RX_temp)

# def temp_rising(lidar,Engine,target_tempera):
#     Diff = temp_difference_cgi_qt(target_tempera, lidar)
#     if Diff < 0:
#         return '雷达温度超限，拔线降温'
#     elif:
#         Diff < 20:
#
#         return '逼近目标温度，调整温槽温度为： '
#
def temperas_record(lidar,Engine1,file_name):
    return 1

if __name__ == '__main__':
    file_name = time.strftime("%Y-%m-%d~%H-%M-%S", time.localtime()) + '.txt'
    print('按下回车开始计时，按下 Ctrl + C 停止计时。')
    Engine1 = Communication('/dev/ttyUSB0', 19200, 0.5)
    Lidar = CGI_CMD(host='192.168.1.201')
    d_figure = dynamic_figure(80,-50,120)
    Lidar_cgi_break = 0
    try:
        Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body']['TempRX'])
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
                    Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body']['TempRX'])
                except:
                    Lidar_cgi_break += 1
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
                f.write('\n')                Lidar_tempera = 60
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
