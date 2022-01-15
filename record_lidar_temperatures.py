# 单纯用来记录雷达温度曲线


import time
from qt_http_sender import CGI_CMD
from refrigerator_communication import Communication,dynamic_figure
from foundation_class import Lydar

def record_gene(lidar_id, length, ld_type):
    if ld_type in ['qt']:
        Lidar = CGI_CMD(host='192.168.1.201')
        try:
            Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body']['TempRX'])
            print('雷达温度：', Lidar_tempera)
        except:
            print('没接雷达:' + ld_type)
            return 0
    else:
        Lidar = Lydar()
        Lidar.__get_binfo_from_udp__()
        if ld_type == 'xt':
            try:
                Lidar_tempera = Lidar.parse_reserved_bits_onerow()['TempA_RFB_1']
                print('雷达温度：', Lidar_tempera)
            except:
                print('没接雷达:' + ld_type)
                return 0
        elif ld_type == 'xtm':
            Lidar_tempera = False
            for i in range(5):
                try:
                    Lidar_tempera = Lidar.parse_reserved_bits_onerow()['TempA_RFB_1']
                    print('雷达温度：', Lidar_tempera)
                except:
                    print('未能连接雷达:' + ld_type)
            if Lidar_tempera == False:
                print('fail')
                return 0
        elif ld_type == 'at':
            try:
                Lidar_tempera = float(Lidar.get_statistic_cgi()['work_temp'])
                print('雷达温度：', Lidar_tempera)
            except:
                print('没接雷达:' + ld_type)
                return 0
    file_name = time.strftime("%Y-%m-%d~%H-%M-%S", time.localtime()) + '_' + lidar_id +'_' + ld_type + '.txt'
    Lidar_cgi_break = 0
    starttime = time.time()
    print('开始')
    while True:
        print('计时: ', round(time.time() - starttime, 0), '秒')  # , end="\r")
        f = open(file_name, 'a')
        f.write('time:' + str(round( time.time()  - starttime, 1) )+ '秒')
        if ld_type == 'qt':
            try:
                Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body']['TempRX'])
            except:
                Lidar_cgi_break += 1
        else:
            if ld_type in ['xt','xtm']:
                try:
                    Lidar_tempera = float(Lidar.parse_reserved_bits_onerow()['TempA_RFB_1'])
                except:
                    Lidar_cgi_break += 1
            elif ld_type == 'at':
                try:
                    Lidar_tempera = float(Lidar.get_statistic_cgi()['work_temp'])
                except:
                    Lidar_cgi_break += 1
        print('Lidar temp is ' + str(Lidar_tempera) + ' ℃')
        f.write(' Lidar_T: ' + str(Lidar_tempera))
        f.write('\n')
        f.close()
        time.sleep(2)
        if length < time.time()  - starttime:
            break

    return Lidar_cgi_break
def record_liquid(length):
    refrigerator_port = '/dev/ttyUSB1'
    communication_break = 0
    Engine1 = Communication(refrigerator_port, 19200, 0.5)
    Engine1.Operation_options(1)
    file_name = time.strftime("%Y-%m-%d~%H-%M-%S", time.localtime()) + '_' + 'liquid' + '.txt'
    starttime = time.time()
    print('开始')
    while True:
        print('计时: ', round(time.time() - starttime, 0), '秒')  # , end="\r")
        f = open(file_name, 'a')
        f.write('time:' + str(round( time.time()  - starttime, 1) )+ '秒')
        try:
            Liquid_tempera = float(Engine1.check_temperature(1))
        except:
            communication_break += 1
        print('Liquid temp is ' + str(Liquid_tempera) + ' ℃')
        f.write(' Lidar_T: ' + str(Liquid_tempera))
        f.write('\n')
        f.close()
        time.sleep(2)
        if length < time.time()  - starttime:
            break
    return communication_break


if __name__ == '__main__':
    break_times_lidar_xt = record_gene('at_d_cjj', 10000, 'at')


    # break_times_liquid = record_liquid(1000)