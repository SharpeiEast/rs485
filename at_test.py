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
        elif temp > t_banchmark and gas_status == 1:
            relayClose(host, port, "1")
            gas_status = 0
            time.sleep(1)
        elif temp > self.ceiling and gas_status == 0:
            relayOpen(host, port, "1")
            gas_status = 2
            time.sleep(1)
        return gas_status

    def target_temp(self, temp, temperas):
        # temperas = [0,10,30,50,70,90,110]
        if len(temperas) != 0:
            if temp > temperas[0]:
                last_temp = temperas.pop(0)
                return temperas, min(last_temp + 30.1, self.ceiling)
            else:
                return temperas, False
        else:
            if temp > self.ceiling:
                return temperas, 0.1
            else:
                return temperas, False

if __name__ == '__main__':
    temperas = [-30, -19.6, 0, 10, 30, 50, 70, 85]  # AT升温范围
    TC = Tempera_control(-40, 90)  # QT安全温度上下限
    gas_sta = 0  # 吹气状态，0表示关闭，1表示开启
    tempera_index = 'TempRX'  # 如果是B样雷达则更改为'TempAvrg'
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
    Engine1.set_temperature(0.1)
    # voltage = 24
    # ser_turnOn(float(voltage), sport=powerSupply_port)
    Lidar = Lydar()
    Lidar.__get_binfo_from_udp__()
    dict_inf = Lidar.get_statistic_cgi()
    d_figure = dynamic_figure(80,-50,120)
    Lidar_cgi_break = 0
    try:
        Lidar_tempera = float(dict_inf['work_temp'])
        print('雷达温度：', Lidar_tempera)
    except:
        print('胖子，你没接雷达')
