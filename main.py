# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


import os
import time
from refrigerator_communication import Communication




if __name__ == '__main__':
    print('按下回车开始计时，按下 Ctrl + C 停止计时。')
    Engine1 = Communication("com5", 19200, 0.5)
    while True:
        input("")  # 如果是 python 2.x 版本请使用 raw_input()
        starttime = time.time()
        # t_list = [30,31,32]
        print('开始')
        try:
            while True:
                print('计时: ', round(time.time() - starttime, 0), '秒')  # , end="\r")
                f = open('record_time.txt', 'a')
                f.write('time:' + str(round( time.time()  - starttime, 1) )+ '秒')
               # f.write('\r\n')
                f.write('\n')
                f.close()
               # t_list, judge,rx_temp = temp_judge(t_list)
                current_tempera = Engine1.check_temperature(1)
                print('Temp is ' + str(current_tempera))
                print('***')
              #  print('t_list: ',t_list)
              #  print(judge)
                print('---')
                f = open('record_tempera.txt', 'a')
                f.write('Temp: ' + str(current_tempera))
                f.write('\n')
                # if judge:
                #     f.write('  Record Point Cloud')
                #     f.write('\r\n')
                f.close()
                time.sleep(2)
        except KeyboardInterrupt:
            print('结束')
            endtime = time.time()
            print('总共的时间为:', round(endtime - starttime, 2), 'secs')
            break
