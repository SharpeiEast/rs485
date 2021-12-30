import json
import time
import requests
from requests.exceptions import ConnectionError, ConnectTimeout

class CGI_CMD():

    def __init__(self, host='192.168.1.201'):
        self.host = host
        self.base_url = 'http://{}/pandar.cgi?'.format(self.host)

    def cgi_get_device_info(self):
        get_param = {'action': 'get',
                     'object': 'device_info'}
        response = requests.get(self.base_url, params=get_param, timeout=1)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_lidar_config(self):
        get_param = {'action': 'get',
                     'object': 'lidar_config'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_ethernet_all(self):
        get_param = {'action': 'get',
                     'object': 'ethernet_all'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_lidar_sync(self):
        get_param = {'action': 'get',
                     'object': 'lidar_sync'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_TimeStatistic(self):
        get_param = {'action': 'get',
                     'object': 'TimeStatistic'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_lidar_range(self):
        get_param = {'action': 'get',
                     'object': 'lidar_data',
                     'key': 'lidar_range'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_lidar_mode(self):
        get_param = {'action': 'get',
                     'object': 'lidar_data',
                     'key': 'lidar_mode'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        body = re['Body']
        return_mode = {'0': 'Last Return', '1': 'Strongest Return', '2': 'Last and Strongest Return', '3': 'First return',
                       '4': 'Last and First Return', '5': 'First and Strongest Return'}[body['lidar_mode']]
        print('Current return mode: {}.'.format(return_mode))
        return return_mode

    def cgi_get_standbymode(self):
        get_param = {'action': 'get',
                     'object': 'lidar_data',
                     'key': 'standbymode'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        body = re['Body']
        mode = {'0': 'in operation', '1': 'standby'}[body['standbymode']]
        print('Current standbymode: {}.'.format(mode))
        return mode

    def cgi_get_workmode(self):
        get_param = {'action': 'get',
                     'object': 'workmode'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_upgrade_log(self):
        get_param = {'action': 'get',
                     'object': 'upgrade_log'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_PTP_lock_offset(self):
        get_param = {'action': 'get',
                     'object': 'PTP_lock_offset'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_get_log_file_list(self):
        get_param = {'action': 'get',
                     'object': 'log_file_list'}
        response = requests.get(self.base_url, params=get_param)
        re = json.loads(response.text)
        print(re)
        return re

    def cgi_set_ipdes(self, ipv4='192.168.1.100', port=2369, gpsport=10110):
        value = '{"IPv4":' + str(ipv4) + ',"Port":' + str(port) + ',"GpsPort":' + str(gpsport) + "}'"
        set_param = {"action": "set",
                     "object": "lidar",
                     "key": "ip_disnation",
                     "value": value}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print(re)

    def cgi_set_trigger_method(self, trigger_method='angle based'):
        """
        @param: trigger method (angle based / time based)
        """
        value = {'ANGLE BASED': '0', 'TIME BASED': '1'}[trigger_method.upper()]
        set_param = {'action': 'set',
                     'object': 'lidar',
                     'key': 'trigger_method',
                     'value': value}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('This is response for setting trigger method: {}'.format(re))

    def cgi_set_rotate_direction(self, rotate_direction='clockwise'):
        """
        @param: rotate direction (clockwise / counterclockwise)
        """
        value = {'CLOCKWISE': '0', 'COUNTERCLOCKWISE': '1'}[rotate_direction.upper()]
        set_param = {'action': 'set',
                     'object': 'lidar',
                     'key': 'rotate_direction',
                     'value': value}
        request = requests.get(self.base_url, params=set_param)
        re = json.loads(request.text)
        print(re)
        time.sleep(5)
        re_get = self.cgi_get_lidar_config()
        rotate_direction = {'0': 'Clockwise', '1': 'Counterclockwise'}[re_get['Body']['RotateDirection']]
        print('Now Rotate Direction: {}'.format(rotate_direction))

    def cgi_set_sync_angle(self, enable_flag=1, sync_angle=180):
        """
        @param1: sync flag (0: Disable, 1: Enable)
        @param2: Sync angle, in units of 1°
        """
        if sync_angle > 360 or sync_angle < 0:
            print("wrong synchronization angle")
        syn_value = '{"sync":' + str(enable_flag) + ',"syncAngle":' + str(sync_angle) + "}'"
        set_p = {
            "action": "set",
            "object": "lidar_sync",
            "key": "sync_angle",
            "value": syn_value
        }
        response = requests.get(self.base_url, params=set_p)
        re = json.loads(response.text)
        print('This is response for set sync angle: {}.'.format(re))

    def cgi_set_return_mode(self, return_mode='last and first return'):
        """
        @param: return mode (last and first return / last return / first return)
        """
        value = {'LAST RETURN': '0', 'STRONGEST RETURN': '1', 'LAST AND STRONGEST RETURN': '2', 'FIRST RETURN': '3',
                 'LAST AND FIRST RETURN': '4', 'FIRST AND STRONGEST RETURN': '5'}[return_mode.upper()]
        set_param = {'action': 'set',
                     'object': 'lidar_data',
                     'key': 'lidar_mode',
                     'value': value}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('This is response for setting return mode: {}.'.format(re))

    def cgi_set_standbymode(self, mode='standby'):
        """
        @param: mode (in operation / standby)
        """
        value = {'IN OPERATION': '0', 'STANDBY': '1'}[mode.upper()]
        set_param = {'action': 'set',
                     'object': 'lidar_data',
                     'key': 'standbymode',
                     'value': value}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('This is response for setting standbymode: {}'.format(re))

    def cgi_set_reboot(self):
        set_param = {'action': 'set',
                     'object': 'reboot'}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('This is response for setting reboot: {}'.format(re))

    def cgi_set_reset(self):
        set_param = {'action': 'set',
                     'object': 'reset'}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('This is response for setting reset: {}'.format(re))

    def cgi_set_PTP_lock_offset(self, offset=2):
        key = str(offset)
        set_param = {'action': 'set',
                     'object': 'PTP_lock_offset',
                     'key': key}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('This is response for setting PTP lock offset: {}'.format(re))

    def cgi_set_spin_speed(self, spin_speed=600):
        value = {'600': '2', '1200': '3'}[str(spin_speed)]
        set_param = {'action': 'set',
                     'object': 'lidar',
                     'key': 'spin_speed',
                     'value': value}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('This is response for setting spin speed: {}'.format(re))

    def cgi_set_retro_multireflection(self, value='disable'):
        value = {'DISABLE': '0', 'ENABLE': '1'}[value.upper()]
        set_param = {'action': 'set',
                     'object': 'lidar_data',
                     'key': 'retro_multireflection',
                     'value': value}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('This is response for setting retro reflection is : {}'.format(re))

    def cgi_post_upgrade(self, file_path):
        ufile = {'file': open(file_path, 'rb')}
        post_url = self.base_url[:-12] + '/upgrade.cgi'
        response = requests.post(post_url, files=ufile, timeout=30)
        re = json.loads(response.text)
        print(re)

    def cgi_set_fov_for_all_chn_cgi(self, min_value, max_value):
        if min_value*10 > 3600 or min_value*10 < 0 or max_value*10 > 3600 or max_value*10 < 0:
            print("wrong input please double check, value should be between 0 and 3600")
        fov_value = {"angle_setting_method": 0, "lidar_range": [min_value*10, max_value*10]}
        timeout = 10
        retry_num = 2
        files = None
        data = json.dumps(fov_value)
        s = requests.Session()
        # a = requests.adapters.HTTPAdapter(max_retries=retry_num)
        # s.mount('http://', a)
        post_url = self.base_url + "action=set&object=lidar_data&key=lidar_range"
        res = s.request('POST', post_url, data=data, files=files, timeout=timeout)
        print(res)
        info = json.loads(res.text)
        info['Connection_Status'] = res.status_code
        return info

    def cgi_set_multi_fov(self, fov_list):
        fov_2_set = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
        len_fov = len(fov_list)
        if len_fov > 5:
            return ''
        else:
            for i in range(len_fov):
                for j in range(2):
                    fov_list[i][j] *= 10
                fov_2_set[i] = fov_list[i]
            fov_value = {"angle_setting_method": 3, "lidar_range": [0, 3600], 'lidar_range_extend': fov_2_set}
            timeout = 10
            retry_num = 2
            files = None
            data = json.dumps(fov_value)
            s = requests.Session()
            # a = requests.adapters.HTTPAdapter(max_retries=retry_num)
            # s.mount('http://', a)
            post_url = self.base_url + "action=set&object=lidar_data&key=lidar_range"
            res = s.request('POST', post_url, data=data, files=files, timeout=timeout)
            print(res)
            assert res.status_code == 200, 'cannot connect to lidar, error code: %s' % (res.status_code)
            info = json.loads(res.text)
            info['Connection_Status'] = res.status_code
            return info

    def cgi_set_channel_config_switch(self, channel_num=48):
        """
        @func: this function is for Zoox only
        @param: int of channel number (48 / 64)
        """
        value = {48: '1', 64: '2'}[channel_num]
        set_param = {'action': 'set',
                     'object': 'lidar_data',
                     'key': 'channel_config_switch',
                     'value': value}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print('Channel Config is switched to {} CHANNELS'.format(channel_num))

    def cgi_get_channel_config_switch(self):
        """
        @func: this function is for Zoox only
        """
        set_param = {'action': 'get',
                     'object': 'lidar_data',
                     'key': 'channel_config_switch',}
        response = requests.get(self.base_url, params=set_param)
        re = json.loads(response.text)
        print(re)

    def cgi_get_factory_monitor(self):
        set_param = {'action': 'get',
                     'object': 'factory_monitor'}
        response = requests.get(self.base_url, params=set_param, timeout=1)
        re = json.loads(response.text)
        return re

#
# if __name__ == '__main__':
#     cgi = CGI_CMD(host='192.168.1.201')
    # cgi.cgi_get_device_info()
    # cgi.cgi_get_lidar_config()
    # cgi.cgi_get_ethernet_all()
    # cgi.cgi_get_lidar_sync()
    # cgi.cgi_get_TimeStatistic()
    # cgi.cgi_get_lidar_range()
    # cgi.cgi_get_lidar_mode()
    # cgi.cgi_get_standbymode()
    # cgi.cgi_get_workmode()
    # cgi.cgi_get_upgrade_log()
    # cgi.cgi_get_PTP_lock_offset()
    # cgi.cgi_get_log_file_list()
    # cgi.cgi_set_ipdes()  # TODO
    # cgi.cgi_set_trigger_method('angle based')
    # cgi.cgi_set_rotate_direction('clockwise')
    # cgi.cgi_set_sync_angle(1, 180)
    # cgi.cgi_set_return_mode('first and strongest return')
    # cgi.cgi_set_standbymode('standby')
    # cgi.cgi_set_reboot()
    # cgi.cgi_set_reset()
    # cgi.cgi_set_PTP_lock_offset(1)
    # cgi.cgi_set_spin_speed(600)
    # cgi.cgi_set_retro_multireflection('enable')
    # cgi.cgi_post_upgrade(r'\\172.16.2.20\qt\TestData\QT-128\4.版本测试\A sample\software_2.0.6_sensor_2.0.6_controller_'
    #                      r'2.0.8_RX_B0_rework\固件\PandarQTPro_sensor_2.0.6_controller_2.0.8.0_software_2.0.6fac_A.patch')
    # cgi.cgi_set_fov_for_all_chn_cgi(60, 180)
    # cgi.cgi_set_multi_fov([[0, 30], [90, 120], [180, 210], [240, 270], [330, 360]])

    # cgi.cgi_get_lidar_mode()

    # cgi.cgi_set_channel_config_switch(48)
    # cgi.cgi_get_channel_config_switch()
    # re = cgi.cgi_get_factory_monitor()
    # print(re['Body'])
    # rx_temp = re['Body']['TempTxAvrg']
    # print(rx_temp)

