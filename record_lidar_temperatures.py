# 单纯用来记录雷达温度曲线


import time
from qt_http_sender import CGI_CMD


def record_gene(lidar_id, length):
    Lidar = CGI_CMD(host='192.168.1.201')
    try:
        Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body']['TempRX'])
        print('雷达温度：', Lidar_tempera)
    except:
        print('没接雷达')
        return 0
    file_name = time.strftime("%Y-%m-%d~%H-%M-%S", time.localtime()) + '_' + lidar_id + '.txt'
    Lidar_cgi_break = 0
    starttime = time.time()
    print('开始')
    while True:
        print('计时: ', round(time.time() - starttime, 0), '秒')  # , end="\r")
        f = open(file_name, 'a')
        f.write('time:' + str(round( time.time()  - starttime, 1) )+ '秒')
        try:
            Lidar_tempera = float(Lidar.cgi_get_factory_monitor()['Body']['TempRX'])
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

break_times = record_gene('0011', 10000)