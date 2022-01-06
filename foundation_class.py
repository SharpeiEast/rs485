import base64
import binascii
import json
import os
import socket
import struct
import sys
import time
from datetime import datetime, timezone

import numpy as np
import pandas as pd
import itertools
import requests
from requests.exceptions import SSLError, ConnectionError
from selenium.common.exceptions import UnexpectedAlertPresentException, NoSuchElementException, WebDriverException

sys.path.append('..')

requests.packages.urllib3.disable_warnings()


class Lydar:

    def __init__(self, host='192.168.1.201', port=2368):
        self.host = host
        self.port = port
        self.data_size = None
        self.header_size = 12
        self.init_hardware_port()
        self.lidar_label = None

    def __get_binfo_from_udp__(self):
        '''
        function for initialization, to get lidar information online
        :return: instance
        '''
        # udp data can only be received while power supplied
        if not self.data_size:
            data = self.read_UDP_data_once()
        else:
            data = self.read_UDP_data_once(new_udp=0)

        self.__parse_func_for_binfo__(data)
        self.__set_cs_flag__()
        self.__set_fac_flag__()
        if self.protocol_version in [64.6, 99.99]:
            if not self.get_device_info_cgi(info_type='udp_sequence'):
                self.set_udp_seq_ena_cgi(ena_value=1)  # enable udp sequence
                self.udp_seq_flag = 1
                self.data_size = self.data_size + 4
                for _ in range(5000):
                    data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
                data = self.read_UDP_data_once(new_udp=0)
                self.__parse_func_for_binfo__(data)

        body_dict = self.parse_udp_body_general(data, data_type='all', info_type='azmth')
        self.body_keys = list(body_dict.keys())
        l2_tail_dict = self.parse_udp_tail_general(data, data_type='all', return_type='dict')
        l2_tail_dict_wo_reserved = self.parse_udp_tail_general(data, data_type='all', return_type='dict_wo_reserveds')
        self.l2_tail_keys = list(l2_tail_dict.keys())
        self.l2_tail_keys_wo_reserveds = list(l2_tail_dict_wo_reserved.keys())

        self.azmth_idx_dict = self.parse_udp_body_general(data, data_type='all', info_type='azmth_idx_dict')
        self.tail_index_dict = self.parse_udp_tail_general(data, data_type='all', return_type='tail_index_dict')
        self.block1_body_info = self.parse_udp_body_general(data, data_type='all',
                                                            info_type='distance_idx_block1_tuple')
        self.block_info = self.parse_udp_body_general(data, data_type='all', info_type='blocks_info')

        self.get_udp_num_psec()
        self.get_standard_resolution()
        self.__init_reserved_info__()
        self.parse_sha_in_udp()
        self.get_lidar_type()
        self.get_temp_range_reboot_time()
        # self.gen_gps_socket()
        # ptc_socket 建立

    def __instance_via_offline__(self, pcap_file):
        '''
        create Lydar instance via pcap file offline, in this case, all cgi methods stays invalid to call. but method
        '' can be used for parsing pcap
        :param pcap_file: pcap file
        :return: None
        '''
        # with open(pcap_file, 'rb') as file_handle:
        #     tmp_data = file_handle.read(2500)
        # first_udp_header_bit = tmp_data.find(b'\xee\xff')
        # second_udp_header_bit = tmp_data[first_udp_header_bit + 2:].find(b'\xee\xff')
        # self.data_size = second_udp_header_bit + 2 - 58
        # data = tmp_data[first_udp_header_bit: first_udp_header_bit + self.data_size]
        data, rpos = self.pick_out_first_udp(pcap_file)
        self.data_size = len(data)
        self.__instance_via_udp_data_offline__(data)
        self.block1_body_info = self.parse_udp_body_general(data, data_type='all',
                                                            info_type='distance_idx_block1_tuple')
        self.__init_reserved_info__(online_flag=0)
        self.get_udp_num_psec(online_flag=0)
        self.get_standard_resolution(online_flag=0)
        self.get_lidar_type()

    def __instance_via_udp_data_offline__(self, data):
        '''
        create Lydar instance via one udp data offline, in this case, all cgi methods stays invalid to call.
        :param data: one single udp data in bytes
        :return: None
        '''
        self.__parse_func_for_binfo__(data)

        body_dict = self.parse_udp_body_general(data, data_type='all', info_type='azmth')
        self.body_keys = list(body_dict.keys())
        l2_tail_dict = self.parse_udp_tail_general(data, data_type='all', return_type='dict')
        l2_tail_dict_wo_reserved = self.parse_udp_tail_general(data, data_type='all', return_type='dict_wo_reserveds')
        self.l2_tail_keys = list(l2_tail_dict.keys())
        self.l2_tail_keys_wo_reserveds = list(l2_tail_dict_wo_reserved.keys())
        self.azmth_idx_dict = self.parse_udp_body_general(data, data_type='all', info_type='azmth_idx_dict')
        self.tail_index_dict = self.parse_udp_tail_general(data, data_type='all', return_type='tail_index_dict')
        self.block1_body_info = self.parse_udp_body_general(data, data_type='all',
                                                            info_type='distance_idx_block1_tuple')
        self.block_info = self.parse_udp_body_general(data, data_type='all', info_type='blocks_info')

    def __parse_func_for_binfo__(self, data):
        self.laser_num, self.block_num, self.first_bck_return, tmp, self.return_num = struct.unpack('<BBBBB',
                                                                                                    data[6:11])
        self.dist_unit = tmp * 0.001
        pv_major, pv_minor = struct.unpack('<BB', data[2:4])
        if pv_minor == 49:
            pv_minor = 4
            self.ab = [29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52,
                       53, 54, 56, 58, 60, 62, 64]
            self.abc = self.ab + [66, 68, 70, 72, 74, 76, 78, 83, 88, 93]
            self.abd = self.ab + [0, 0, 0, 99, 104, 109, 114, 117, 120, 123]
            self.laser_id_list = [self.abd, self.abc, self.abd]
        elif pv_minor == 50:
            pv_minor = 5
            self.ab = [20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
                       44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 60, 62, 64]
            self.abc = self.ab + [66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 89, 94, 96]
            self.abd = self.ab + [0, 0, 0, 0, 105, 110, 116, 118, 120, 122, 124, 126, 128]
            self.laser_id_list = [self.abd, self.abc, self.abd]
        #     single: cdcdcdcd, dual: ccddccdd
        self.protocol_version = int(pv_major) + round(int(pv_minor) / 10, 1)
        self.flag = struct.unpack('<B', data[11:12])[0]
        if self.protocol_version == 4.1:
            self.flag = 17
        self.data_size = len(data)
        self.weight_factor = (self.flag >> 5) % 2
        self.conf_flag = (self.flag >> 4) % 2  # value of bit4
        self.sign_flag = (self.flag >> 3) % 2  # value of bit3
        self.l2_flag = (self.flag >> 2) % 2  # value of bit2
        self.imu_flag = (self.flag >> 1) % 2  # value of bit1
        self.udp_seq_flag = self.flag % 2  # value of bit0
        self.pt_byte_num = 3 + self.conf_flag
        self.utm_year_offset = 1970
        if self.protocol_version in [1.3]:
            self.body_size = self.block_num * (self.laser_num * self.pt_byte_num + 2)
            self.tail_size = 24 + 4 * self.udp_seq_flag
            self.tail_start_bit = self.header_size + self.body_size + 17 * self.l2_flag
            self.cs_flag = 0
        elif self.protocol_version in [1.4, 1.45, 3.2, 3.4, 3.5, 3.22]:
            self.body_size = self.block_num * (self.laser_num * self.pt_byte_num + 2) + 4
            self.tail_size = 26 + 22 * self.imu_flag + 4 * self.udp_seq_flag + self.sign_flag * 32 + 4 * self.udp_seq_flag
            self.tail_start_bit = self.header_size + self.body_size + 17 * self.l2_flag
        elif self.protocol_version in [4.3]:
            self.body_size = self.block_num * (self.laser_num * self.pt_byte_num + 3) + 4
            self.tail_size = 32 + 22 * self.imu_flag + 4 * self.udp_seq_flag + self.sign_flag * 32 + 4
            self.tail_start_bit = self.header_size + self.body_size + 17 * self.l2_flag
        elif self.protocol_version in [64.6]:
            self.laser_num = pv_major
            self.block_num = pv_minor
            self.first_bck_return = 0
            self.return_num = 2
            self.dist_unit = 0.004
            self.header_size = 8
            self.flag = 0
            self.conf_flag = (self.flag >> 4) % 2  # value of bit4
            self.sign_flag = (self.flag >> 3) % 2  # value of bit3
            self.l2_flag = (self.flag >> 2) % 2  # value of bit2
            self.imu_flag = (self.flag >> 1) % 2  # value of bit1
            self.udp_seq_flag = self.flag % 2  # value of bit0
            self.pt_byte_num = 3 + self.conf_flag
            self.body_size = self.block_num * (self.laser_num * self.pt_byte_num + 2)
            self.tail_size = self.data_size - self.body_size - self.header_size
            self.tail_start_bit = self.header_size + self.body_size + 17 * self.l2_flag
            self.cs_flag = 0
            self.utm_year_offset = 2000
        elif self.protocol_version in [6.1, 3.1, 4.1]:
            self.conf_flag = 1  # bit4 is wrong!!
            self.pt_byte_num = 3 + self.conf_flag
            self.body_size = self.block_num * (self.laser_num * 4 + 2)
            self.tail_size = 24 + 4 * self.udp_seq_flag
            self.tail_start_bit = self.header_size + self.body_size + 17 * self.l2_flag
            self.cs_flag = 0
            self.utm_year_offset = 1970
        else:
            self.protocol_version = 99.99  # only for 40M,P
            self.laser_num = 40
            self.block_num = 10
            self.first_bck_return = 0
            self.return_num = 2
            self.header_size = 0
            self.flag = 0
            self.conf_flag = (self.flag >> 4) % 2  # value of bit4
            self.sign_flag = (self.flag >> 3) % 2  # value of bit3
            self.l2_flag = (self.flag >> 2) % 2  # value of bit2
            self.imu_flag = (self.flag >> 1) % 2  # value of bit1
            self.udp_seq_flag = self.flag % 2  # value of bit0
            self.pt_byte_num = 3 + self.conf_flag
            # self.body_size = self.block_num * (self.laser_num * self.pt_byte_num + 2)
            self.body_size = 1240
            self.tail_size = self.data_size - self.body_size - self.header_size
            self.tail_start_bit = self.header_size + self.body_size + 17 * self.l2_flag
            self.cs_flag = 0
            self.udp_num_psec = 3600
            self.utm_year_offset = 2000
            self.dist_unit = 0.004

        if self.conf_flag:
            self.dist_int_type = np.dtype([('distance', 'uint16'), ('intensity', 'uint8'), ('confidence', 'uint8')])
        else:
            self.dist_int_type = np.dtype([('distance', 'uint16'), ('intensity', 'uint8')])

    def __set_fac_flag__(self):
        # self.cookies is also filled after this step
        if self.protocol_version == [3.2, 3.4, 3.5, 3.22]:
            reg_value = "40022800"
        elif self.protocol_version == 3.1:
            reg_value = "400228ce"
        elif self.protocol_version in [4.1, 4.3]:
            reg_value = "8090f004"
        else:
            reg_value = "43c082dc"
        get_p = {
            "action": "get",
            "object": "register",
            "key": "down",
            "value": reg_value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        # todo: release vesrion has no factory page
        if self.cs_flag == 1:
            response = requests.get(self.base_url, get_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p)
        r = json.loads(response.text)
        # print(r)
        if "not support" in r["Head"]["Message"] or '3' in r["Head"]["ErrorCode"]:
            self.fac_flag = 0
        elif "Success" in r["Head"]["Message"]:
            self.fac_flag = 1
        else:
            # print(r)
            self.fac_flag = -1
            print("factory or release, that is a question!")

    def __get_cp_sign_flag_p128_cs__(self):
        # todo
        data = self.read_UDP_data_once()
        if len(data) > self.body_size:
            self.cp_sign_size = len(data) - self.body_size
            print("point cloud signifiture size is %s" % self.cp_sign_size)
            self.body_size = len(data)
            self.tail_size = self.tail_size + self.cp_sign_size
            self.cp_sign_flag = 1

    def __get_model_and_laser_num_ptc__(self):
        r = self.ptc_sender(0x07, None)[0]
        self.laser_num = int.from_bytes(r["response_payload"][108:109], 'big')
        self.model_id = int.from_bytes(r["response_payload"][106:107], 'big')

    def __parse_function_safety_v14__(self, data_all, data_type='all', return_type='dict'):
        if self.l2_flag == 0:
            print('function safety excluded')
            return None
        elif data_type == 'safety+tail':
            data_l2 = data_all[0:17]
        elif data_type == 'all':
            data_l2 = data_all[self.header_size + self.body_size:self.header_size + self.body_size + 17]
        elif data_type == 'safety':
            data_l2 = data_all
        else:
            print('wrong data type input!')
            return None

        FS_version = struct.unpack('<B', data_l2[0:1])[0]
        raw_bits1 = struct.unpack('<B', data_l2[1:2])[0]
        lidar_state = raw_bits1 >> 5
        code_type = (raw_bits1 >> 3) % 4  # 1: current; 2: history
        roll_cnt = raw_bits1 % 8
        raw_bits2 = struct.unpack('<B', data_l2[2:3])[0]
        total_cd_num = raw_bits2 >> 4
        flt_cd_id = raw_bits2 % 16
        flt_code = hex(struct.unpack('<H', data_l2[3:5])[0])
        reserved = data_l2[5:13]
        crc2 = struct.unpack('<I', data_l2[13:17])[0]
        l2_dict_keys = ['FS_version', 'lidar_state', 'code_type', 'rolling_counter', 'total_code_num', 'fault_code_id',
                        'fault_code', 'crc2']
        l2_list = [FS_version, lidar_state, code_type, roll_cnt, total_cd_num, flt_cd_id, flt_code, crc2]
        l2_dict = dict(zip(l2_dict_keys, l2_list))
        if return_type == "list":
            return l2_list
        elif return_type == "dict":
            return l2_dict
        else:
            print("wrong input type!")
            return None

    def __parse_udp_tail_v13_qt__(self, data_all, return_type="list"):
        # return tail information in form of list or dictionary, "list"/"dict"
        data_tail = data_all[0 - self.tail_size:]
        reserved1 = struct.unpack('<H', data_tail[0:2])[0]
        reserved1_id = struct.unpack('<B', data_tail[2:3])[0]
        reserved2 = struct.unpack('<H', data_tail[3:5])[0]
        reserved2_id = struct.unpack('<B', data_tail[5:6])[0]
        # 高温shutdown 1位
        high_t_shutdown = (struct.unpack('<B', data_tail[6:7])[0]) % 8
        # 时分复用的reserved 3位（误码率）
        error_code = struct.unpack('<H', data_tail[7:9])[0]
        error_code_id = struct.unpack('<B', data_tail[9:10])[0]
        # 转速 2位
        motor_speed = struct.unpack('<H', data_tail[10:12])[0]
        # 时间戳 4位
        pcap_timestamp = gen_timestamp(data_tail[12:16])
        # return mode
        return_mode = struct.unpack('<B', data_tail[16:17])[0]
        # 工厂信息 1位
        factory_info = struct.unpack('<B', data_tail[17:18])[0]
        # UTC 6位
        utc_time = gen_timestamp(data_tail[18:24])
        # udp sequence number 4 bytes
        udp_sequence = None

        self.resolution_mode = "high" * (high_t_shutdown == 0) + "standard" * (high_t_shutdown == 2) + \
                               "shutdown" * (high_t_shutdown == 1) + "eco" * (high_t_shutdown == 3)
        self.spin_spd = round(motor_speed, -2)
        # temp_list = [0, 2, 3, 5, 6, 7, 9, 10, 12, 16, 17, 18, (self.tail_start_bit+1)*-1]
        idx_tpl_list = [(0, 2), (2, 1), (3, 2), (5, 1), (6, 1), (7, 2), (9, 1), (10, 2), (12, 4), (16, 1), (17, 1),
                        (18, 6), None]
        if self.udp_seq_flag:
            udp_sequence = struct.unpack('<I', data_tail[24:28])[0]
            # temp_list = [0, 2, 3, 5, 6, 7, 9, 10, 12, 16, 17, 18, 24]
            idx_tpl_list = [(0, 2), (2, 1), (3, 2), (5, 1), (6, 1), (7, 2), (9, 1), (10, 2), (12, 4), (16, 1), (17, 1),
                            (18, 6), (24, 4)]
        index_list = [(i + self.tail_start_bit, j) for i, j in idx_tpl_list]
        tail_list = [reserved1, reserved1_id, reserved2, reserved2_id, high_t_shutdown, error_code, error_code_id,
                     motor_speed,
                     pcap_timestamp, return_mode, factory_info, utc_time, udp_sequence]
        dict_keys = ["reserved1", "reserved1_id", "reserved2", "reserved2_id", "high_t_shutdown", "error_code",
                     "error_code_id",
                     "motor_speed", "pcap_timestamp", "return_mode", "factory_info", "utc_time", "udp_sequence"]
        tail_dict = dict(zip(dict_keys, tail_list))
        tail_dict_wo_reserveds = dict(zip(dict_keys[6:], tail_list[6:]))
        tail_index_dict = dict(zip(dict_keys, index_list))
        if return_type == "list":
            return tail_list
        elif return_type == "dict":
            return tail_dict
        elif return_type == "tail_index_dict":
            return tail_index_dict
        elif return_type == "dict_wo_reserveds":
            return tail_dict_wo_reserveds
        else:
            print("wrong input type!")
            return None

    def __parse_udp_tail_xt__(self, data_all, return_type="list"):
        data_tail = data_all[0 - self.tail_size:]
        reserved1 = struct.unpack('<H', data_tail[0:2])[0]
        reserved1_id = struct.unpack('<B', data_tail[2:3])[0]
        reserved2 = struct.unpack('<H', data_tail[3:5])[0]
        reserved2_id = struct.unpack('<B', data_tail[5:6])[0]
        # 时分复用的reserved 3位（误码率）
        error_code = struct.unpack('<H', data_tail[6:8])[0]
        error_code_id = struct.unpack('<B', data_tail[8:9])[0]
        # 高温shutdown 1位
        high_t_shutdown = (struct.unpack('<B', data_tail[9:10])[0]) % 8
        # return mode
        return_mode = struct.unpack('<B', data_tail[10:11])[0]
        # 转速 2位
        motor_speed = struct.unpack('<H', data_tail[11:13])[0]
        # UTC 6位
        utc_time = gen_timestamp(data_tail[13:19])
        # 时间戳 4位
        pcap_timestamp = gen_timestamp(data_tail[19:23])
        # 工厂信息 1位
        factory_info = struct.unpack('<B', data_tail[23:24])[0]

        self.resolution_mode = "high" * (high_t_shutdown == 0) + "standard" * (high_t_shutdown == 2) + \
                               "shutdown" * (high_t_shutdown == 1) + "eco" * (high_t_shutdown == 3)
        self.spin_spd = round(motor_speed, -2)
        # udp sequence number 4 bytes
        udp_sequence = None
        temp_list = [0, 2, 3, 5, 6, 8, 9, 10, 11, 13, 19, 23, (self.tail_start_bit + 1) * -1]
        idx_tpl_list = [(0, 2), (2, 1), (3, 2), (5, 1), (6, 2), (8, 1), (9, 1), (10, 1), (11, 2), (13, 6), (19, 4),
                        (23, 1), None]
        if self.udp_seq_flag:
            udp_sequence = struct.unpack('<I', data_tail[24:28])[0]
            temp_list = [0, 2, 3, 5, 6, 8, 9, 10, 11, 13, 19, 23, 24]
            idx_tpl_list = [(0, 2), (2, 1), (3, 2), (5, 1), (6, 2), (8, 1), (9, 1), (10, 1), (11, 2), (13, 6), (19, 4),
                            (23, 1), (24, 4)]
        index_tpl_list = [(i + self.tail_start_bit, j) for i, j in idx_tpl_list]

        tail_list = [reserved1, reserved1_id, reserved2, reserved2_id, error_code, error_code_id, high_t_shutdown,
                     return_mode,
                     motor_speed, utc_time, pcap_timestamp, factory_info, udp_sequence]
        dict_keys = ["reserved1", "reserved1_id", "reserved2", "reserved2_id", "error_code", "error_code_id",
                     "high_t_shutdown", "return_mode",
                     "motor_speed", "utc_time", "pcap_timestamp", "factory_info", "udp_sequence"]
        tail_dict = dict(zip(dict_keys, tail_list))
        tail_dict_wo_reserveds = dict(zip(dict_keys[6:], tail_list[6:]))

        tail_index_dict = dict(zip(dict_keys, index_tpl_list))
        if return_type == "list":
            return tail_list
        elif return_type == "dict":
            return tail_dict
        elif return_type == "tail_index_dict":
            return tail_index_dict
        elif return_type == "dict_wo_reserveds":
            return tail_dict_wo_reserveds
        else:
            print("wrong input type!")
            return None

    def __parse_udp_tail_v14__(self, data_all, data_type='all', return_type="list"):
        # data_type: 'all', 'tail', 'safety+tail'; for 'safety+tail' return_type can only set as "dict"
        if data_type == 'all' and not self.l2_flag:
            data_tail = data_all[self.tail_start_bit:]
        elif data_type == 'all' and self.l2_flag:
            # print('both safety and tail data will be parsed, return data in form of dictionary')
            if 'dict' not in return_type:
                return_type = 'dict'
            data_tail = data_all[self.tail_start_bit:]
            data_l2 = data_all[self.header_size + self.body_size:self.header_size + self.body_size + 17]
            l2_dict = self.__parse_function_safety_v14__(data_l2, data_type='safety', return_type='dict')
        elif data_type == 'safety+tail' and self.l2_flag:
            # print('both safety and tail data will be parsed, return data in form of dictionary')
            if 'dict' not in return_type:
                return_type = 'dict'
            data_l2 = data_all[0:17]
            data_tail = data_all[17:]
            l2_dict = self.__parse_function_safety_v14__(data_l2, data_type='safety', return_type=return_type)
        elif data_type == 'safety+tail' and not self.l2_flag:
            print('this lidar has no function safety data, wrong data type input')
            return None
        elif data_type == 'tail':
            data_tail = data_all
        else:
            print('wrong data type input!')
            return None

        temp_list = [0, 2, 3, 5, 6, 8, 9, 11, 12, 13, 15, 21, 25, 26]
        idx_tpl_list = [(temp_list[i], temp_list[i + 1] - temp_list[i]) for i in range(len(temp_list) - 1)]
        idx_tpl_list.append((26, 4))

        # index_list = [i+self.tail_start_bit for i in temp_list]
        reserved1 = struct.unpack('<H', data_tail[0:2])[0]
        reserved1_id = struct.unpack('<B', data_tail[2:3])[0]
        # 如果是老的雷达，则temp2数据无意义
        reserved2 = struct.unpack('<H', data_tail[3:5])[0]
        reserved2_id = struct.unpack('<B', data_tail[5:6])[0]
        # 时分复用的reserved 3位（误码率）
        error_code = struct.unpack('<H', data_tail[6:8])[0]
        error_code_id = struct.unpack('<B', data_tail[8:9])[0]
        # Aziumuth flag
        azimuth_flag = struct.unpack('<H', data_tail[9:11])[0]
        # 高温shutdown 1位
        high_t_shutdown = struct.unpack('<B', data_tail[11:12])[0]
        # return mode 1位
        return_mode = struct.unpack('<B', data_tail[12:13])[0]
        # 转速 2位
        motor_speed = struct.unpack('<H', data_tail[13:15])[0]
        # UTC 6位
        utc_time = gen_timestamp(data_tail[15:21])
        # 时间戳 4位
        pcap_timestamp = gen_timestamp(data_tail[21:25])
        # 工厂信息 1位
        factory_info = struct.unpack('<B', data_tail[25:26])[0]
        # udp sequence number 4 bytes
        udp_sequence = struct.unpack('<I', data_tail[26:30])[0]

        self.resolution_mode = "high" * (high_t_shutdown == 0) + "standard" * (high_t_shutdown == 2) + \
                               "shutdown" * (high_t_shutdown == 1) + "eco" * (high_t_shutdown == 3)
        self.spin_spd = round(motor_speed, -2)
        # imu info 26 bytes
        if self.protocol_version in [1.4, 1.45]:
            imu_info = struct.unpack("<HHHIHHHHHHI", data_tail[30:56])
            self.imu_acc_unit = imu_info[1] / 1e3  # mG
            self.imu_vel_unit = imu_info[2] / 1e2  # mdps
        tail_list = [reserved1, reserved1_id, reserved2, reserved2_id, error_code, error_code_id, azimuth_flag,
                     high_t_shutdown, return_mode, motor_speed, utc_time, pcap_timestamp, factory_info, udp_sequence]
        if self.protocol_version in [3.2, 3.4, 3.5, 3.22]:
            dict_keys = ["reserved1", "reserved1_id", "reserved2", "mode_flag", "error_code", "error_code_id",
                         "azimuth_flag", "high_t_shutdown", "return_mode", "motor_speed", "utc_time", "pcap_timestamp",
                         "factory_info", "udp_sequence"]
        else:
            dict_keys = ["reserved1", "reserved1_id", "reserved2", "reserved2_id", "error_code", "error_code_id",
                         "azimuth_flag", "high_t_shutdown", "return_mode", "motor_speed", "utc_time", "pcap_timestamp",
                         "factory_info", "udp_sequence"]
        tail_dict = dict(zip(dict_keys, tail_list))
        tail_dict_wo_reserveds = dict(zip(dict_keys[6:], tail_list[6:]))

        index_tpl_list = [(i + self.tail_start_bit, j) for i, j in idx_tpl_list]
        tail_index_dict = dict(zip(dict_keys, index_tpl_list))
        if return_type == "list":
            return tail_list
        elif return_type == "dict":
            l2_dict.update(tail_dict)
            return l2_dict
        elif return_type == "tail_index_dict":
            return tail_index_dict
        elif return_type == "dict_wo_reserveds":
            l2_dict.update(tail_dict_wo_reserveds)
            return l2_dict
        else:
            print("wrong input type!")
            return None

    def __parse_udp_tail_v43__(self, data_all, data_type='all', return_type="list"):
        # data_type: 'all', 'tail', 'safety+tail'; for 'safety+tail' return_type can only set as "dict"
        if data_type == 'all' and not self.l2_flag:
            data_tail = data_all[self.tail_start_bit:self.tail_start_bit + 40]
        elif data_type == 'safety+tail' and not self.l2_flag:
            print('this lidar has no function safety data, wrong data type input')
            return None
        elif data_type == 'tail':
            data_tail = data_all
        else:
            print('wrong data type input!')
            return None

        temp_list = [0, 2, 3, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 24, 25, 26, 32, 36]
        idx_tpl_list = [(temp_list[i], temp_list[i + 1] - temp_list[i]) for i in range(len(temp_list) - 1)]
        idx_tpl_list.append((36, 4))

        reserved1, reserved1_id, reserved2, reserved2_id, high_t_shutdown, reserved3, reserved3_id, reserved4, \
        reserved5, reserved6, reserved7, reserved8, reserved9, reserved10, reserved11, motor_spd_raw, pcap_timestamp, \
        return_mode, factory_info, utc_y, utc_m, utc_d, utc_hour, utc_min, utc_sec, \
        udp_sequence, tail_crc = struct.unpack('<HBHBBHBBBBBBBBBhIBBBBBBBBII', data_tail)

        motor_speed = motor_spd_raw * 0.1
        # UTC 6位
        if utc_y < 100:
            n_y = utc_y + self.utm_year_offset
        else:
            n_y = utc_y + self.utm_year_offset - 70
        tmp = [n_y, utc_m, utc_d, utc_hour, utc_min, utc_sec] + [0, 0, 0]
        utc_time = time.mktime(time.struct_time(tmp))

        tail_list = [reserved1, reserved1_id, reserved2, reserved2_id, high_t_shutdown, reserved3, reserved3_id,
                     reserved4, reserved5, reserved6, reserved7, reserved8, reserved9, reserved10, reserved11,
                     motor_speed, pcap_timestamp, return_mode, factory_info, utc_time, udp_sequence, tail_crc]
        dict_keys = ["reserved1", "reserved1_id", "reserved2", "reserved2_id", "high_t_shutdown", "reserved3",
                     "reserved3_id", "reserved4", "reserved5", "reserved6", "reserved7", "reserved8", "reserved9",
                     "reserved10", "reserved11", "motor_speed",
                     "pcap_timestamp", "return_mode", "factory_info", "utc_time", "udp_sequence", "tail_crc"]

        self.resolution_mode = "high" * (high_t_shutdown == 0) + "standard" * (high_t_shutdown == 2) + \
                               "shutdown" * (high_t_shutdown == 1) + "eco" * (high_t_shutdown == 3)
        self.spin_spd = round(motor_speed, -2)

        tail_dict = dict(zip(dict_keys, tail_list))
        tail_dict_wo_reserveds = dict(zip(dict_keys[6:], tail_list[6:]))

        index_tpl_list = [(i + self.tail_start_bit, j) for i, j in idx_tpl_list]
        tail_index_dict = dict(zip(dict_keys, index_tpl_list))

        if return_type == "list":
            return tail_list
        elif return_type == "dict":
            return tail_dict
        elif return_type == "tail_index_dict":
            return tail_index_dict
        elif return_type == "dict_wo_reserveds":
            return tail_dict_wo_reserveds
        else:
            print("wrong input type!")
            return None

    def __parse_udp_tail_unnmb__(self, data_all, return_type="list"):
        # if self.tail_size == 56:
        #     data_tail = data_all[0 - self.tail_size:]
        # else:
        #     data_tail = data_all[0 - self.tail_size:56 - self.tail_size]
        data_tail = data_all[0 - self.tail_size:]
        # reserved id-value 3 bits
        reserved1 = struct.unpack('<H', data_tail[0:2])[0]
        reserved1_id = struct.unpack('<B', data_tail[2:3])[0]
        reserved2 = struct.unpack('<H', data_tail[3:5])[0]
        # 高温shutdown 1位
        high_t_shutdown = struct.unpack('<B', data_tail[5:6])[0]
        crc_code = struct.unpack('<H', data_tail[6:8])[0]
        # 转速 2位
        motor_speed = struct.unpack('<H', data_tail[8:10])[0]
        # 时间戳 4位
        pcap_timestamp = gen_timestamp(data_tail[10:14])
        # return mode 1位
        return_mode = struct.unpack('<B', data_tail[14:15])[0]
        # 工厂信息 1位
        factory_info = struct.unpack('<B', data_tail[15:16])[0]
        # UTC 6位
        utc_time = gen_timestamp(data_tail[16:22])
        self.resolution_mode = "high" * (high_t_shutdown == 0) + "standard" * (high_t_shutdown == 2) + \
                               "shutdown" * (high_t_shutdown == 1) + "eco" * (high_t_shutdown == 3)
        self.spin_spd = round(motor_speed, -2)
        if len(data_tail) == 26:
            udp_sequence = struct.unpack('<I', data_tail[22:26])[0]
            tail_list = [reserved1, reserved1_id, reserved2, high_t_shutdown, crc_code, motor_speed, pcap_timestamp,
                         return_mode, factory_info, utc_time,
                         udp_sequence]
            dict_keys = ["reserved1", "reserved1_id", "reserved2", "high_t_shutdown", "error_code", "motor_speed",
                         "pcap_timestamp",
                         "return_mode", "factory_info", "utc_time", "udp_sequence"]
            temp_list = [0, 2, 3, 5, 6, 8, 10, 14, 15, 16, 22]
            tail_tpl_list = [(temp_list[i], temp_list[i + 1] - temp_list[i]) for i in range(len(temp_list) - 1)]
            tail_tpl_list.append((22, 4))
        elif len(data_tail) == 22:
            tail_list = [reserved1, reserved1_id, reserved2, high_t_shutdown, crc_code, motor_speed, pcap_timestamp,
                         return_mode, factory_info, utc_time]
            dict_keys = ["reserved1", "reserved1_id", "reserved2", "high_t_shutdown", "error_code", "motor_speed",
                         "pcap_timestamp", "return_mode",
                         "factory_info", "utc_time"]
            temp_list = [0, 2, 3, 5, 6, 8, 10, 14, 15, 16]
            tail_tpl_list = [(temp_list[i], temp_list[i + 1] - temp_list[i]) for i in range(len(temp_list) - 1)]
            tail_tpl_list.append((16, 6))
        else:
            print("this lidar supposed to be Pandar64P, but udp tail parse failed")
        tail_dict = dict(zip(dict_keys, tail_list))
        # tail_dict_wo_reserveds = dict(zip(dict_keys[1:], tail_list[1:]))
        index_tpl_list = [(i + self.tail_start_bit, j) for i, j in tail_tpl_list]
        tail_index_dict = dict(zip(dict_keys, index_tpl_list))
        if return_type == "list":
            return tail_list
        elif return_type == "dict":
            return tail_dict
        elif return_type == "tail_index_dict":
            return tail_index_dict
        elif return_type == "dict_wo_reserveds":
            return tail_dict
        else:
            print("wrong input type!")
            return None

    def __set_cs_flag__(self):
        # action=set&object=lidar_data&key=security_code&value=921223'
        get_p = {
            "action": "get",
            "object": "lidar_config"
        }
        try:
            base_url = 'https://{}'.format(self.host)
            response = requests.get(base_url, timeout=2)
        except SSLError:
            base_url = 'https://{}/pandar.cgi?'.format(self.host)
            self.cs_flag = 1
            self.release = 1
        except (requests.exceptions.ConnectTimeout, ConnectionError):
            base_url = 'http://{}/pandar.cgi?'.format(self.host)
            response = requests.get(base_url, get_p)
            r = json.loads(response.text)
            if 'authorization failed' in r['Head']['Message']:
                print('this lidar has cyber security, login cookie needed')
                self.cs_flag = 1
            elif r['Head']['ErrorCode'] == '0':
                self.cs_flag = 0
            else:
                self.cs_flag = -1
            self.release = 0
        self.base_url = base_url
        self.url = self.base_url[:-12]

    def __init_reserved_info__(self, online_flag=1):
        if self.protocol_version in [1.3, 1.4, 1.45]:
            reserved_key = 'reserved1_id'
            self.temp_dict = {0: 'Temp_RFB_1', 2: 'Temp_TMB_1', 3: 'TempA_CB_3', 4: 'TempA_CB_5', 5: 'TempA_RFB_1',
                              7: 'TempA_TMB_1', 8: 'Temp_WCT', 9: 'Temp_TMB_FPGA', 44: 'TempA_WPR', 53: "Temp_RBB_1"}
            self.volt_dict = {26: 'HV_out', 27: 'HV_apd'}
            self.sha_dict = {36: 'SHA_1', 37: 'SHA_2', 38: 'SHA_3', 39: 'SHA_4'}
            self.para_sha_dict = {40: 'Para_SHA_1', 41: 'Para_SHA_2', 42: 'Para_SHA_3', 43: 'Para_SHA_4'}
            self.chr_dict = {}
            self.temp_conv = self.temp_conv_v45
            self.calc_hv = self.calc_hv_v45
            self.reserved_stctstr = '<hB'
            if self.protocol_version in [1.45]:
                self.temp_dict = {0: 'Temp_RFB_1', 2: 'Temp_TMB_1', 66: 'TempD_CB_2', 67: 'TempD_CB_3', 4: 'TempA_CB_5',
                                  5: 'TempA_RFB_1', 7: 'TempA_TMB_1', 8: 'Temp_WCT', 9: 'Temp_TMB_FPGA',
                                  44: 'TempA_WPR', 53: "Temp_RBB_1", 65: 'TempD_LBB_1'}
        elif self.protocol_version in [64.6, 99.99]:
            reserved_key = 'reserved1_id'
            self.temp_dict = {0: 'Temp_L_MON', 1: 'Temp_R_MON', 4: 'Temp_RT2', 5: 'Temp_RT3', 6: 'Temp_RT4',
                              7: 'Temp_RT5'}
            self.volt_dict = {2: 'HV_out', 3: 'HV_apd'}
            self.sha_dict = {12: 'SHA_1', 13: 'SHA_2', 14: 'SHA_3', 15: 'SHA_4'}
            self.para_sha_dict = {16: 'Para_SHA_1', 17: 'Para_SHA_2', 18: 'Para_SHA_3', 19: 'Para_SHA_4'}
            self.chr_dict = {}
            self.temp_conv = self.temp_conv_p64
            self.calc_hv = self.calc_hv_p64
            self.reserved_stctstr = '<HB'
        elif self.protocol_version in [4.1, 4.3]:
            reserved_key = 'reserved2_id'
            # self.temp_dict = {1: 'Temp_FPGA', 4: 'TX_TEMP1', 5: 'TX_TEMP2', 6: 'RX_TEMP1', 7: 'RX_TEMP2',
            #                   8: 'MB_TEMP1', 9: 'MB_TEMP2', 14: 'SIPM_NHV_CUR', 19: 'MBAUX_TEMP'}
            # self.volt_dict = {0: 'VCC_PSINTFP', 2: 'VCC_INT', 3: 'VCC_AUX', 10: 'VCC_0.85V', 11: "TESTVOL",
            #                   12: 'LASER_HV_VOL', 13: 'SIPM_NHV_VOL',
            #                   15: 'VCC24P_VOL', 16: 'RX12.0P_VOL', 17: 'PWRIN_VOL', 18: 'VCC5.0P_VOL'}
            # self.sha_dict = {37: 'SHA_1', 36: 'SHA_2', 39: 'SHA_3', 38: 'SHA_4'}
            # self.para_sha_dict = {41: 'Para_SHA_1', 40: 'Para_SHA_2', 43: 'Para_SHA_3', 42: 'Para_SHA_4'}
            self.temp_dict = {23: 'TEMP_TX1', 24: 'TEMP_RX2', 25: 'TEMP_TX2', 26: 'TEMP_RX1', 27: 'TEMP_MB1',
                              28: 'TEMP_PB',
                              29: 'TEMP_MB2', 30: 'TEMP_HOT', 31: 'FPGA_TEMP'}
            self.volt_dict = {0: 'VCC_0p85', 1: 'VCC_1p1', 2: 'VCC_1p2', 3: 'VCC_2p5', 4: 'VCC_0p625', 5: 'VCC_1p0',
                              6: 'VCC_3p0', 8: 'LASER_HV', 9: 'SIPM_VOL', 10: 'SIPM_CUR', 11: 'PWRIN_VOL',
                              12: 'RX12.0P_VOL',
                              13: 'VCC_5P', 14: 'PWRIN', 15: 'MD_CUR', 16: 'VCC_AUX', 17: 'VCC_HDBANK', 18: 'VCC_INT',
                              19: 'VCC_PSINTLP', 20: 'VCC_PSINTFP', 21: 'VCC_PSAUX', 22: 'VCC_BRAM', 56: 'HOT_RES_CUR',
                              73: 'PWRIN_CUR', 74: 'HEAT_VOL', 75: 'Mean_Power'}
            # self.sha_dict = {49: 'SHA_1', 48: 'SHA_2', 51: 'SHA_3', 50: 'SHA_4'}
            # self.para_sha_dict = {53: 'Para_SHA_1', 52: 'Para_SHA_2', 55: 'Para_SHA_3', 54: 'Para_SHA_4'}
            self.sha_dict = {51: 'SHA_1', 50: 'SHA_2', 49: 'SHA_3', 48: 'SHA_4'}
            self.para_sha_dict = {55: 'Para_SHA_4', 54: 'Para_SHA_2', 53: 'Para_SHA_3', 52: 'Para_SHA_4'}
            self.chr_dict = {32: 'FPGA_Version_4', 33: 'FPGA_Version_3', 34: 'FPGA_Version_2', 35: 'FPGA_Version_1',
                             36: 'soft_version_4', 37: 'soft_version_3', 38: 'soft_version_2', 39: 'soft_version_1',
                             40: 'para1_version_4', 41: 'para1_version_3', 42: 'para1_version_2', 43: 'para1_version_1',
                             44: 'para2_version_4', 45: 'para2_version_3', 46: 'para2_version_2', 47: 'para2_version_1',
                             254: 'TIME_DIV_MULT_2', 255: 'TIME_DIV_MULT_1'}
            self.temp_conv = self.transfer_value_at_qt128
            self.calc_hv = self.transfer_value_at_qt128
            self.reserved_stctstr = '<hB'
        elif self.protocol_version in [3.1]:
            reserved_key = 'reserved1_id'
            self.temp_dict = {0: 'Temp_web', 1: 'Top_circuit_RT1', 2: 'Top_circuit_RT2', 3: 'Laser_emitB_RT1',
                              4: 'Laser_emitB_RT2',
                              6: 'Receive_B_RT1', 9: 'B_circuit_RT1', 10: 'B_circuit_RT2'}
            self.volt_dict = {5: 'HV_TX', 7: 'HV_SiPM', 8: 'HV_Vbias', 11: 'dwn_BuckBoost_IA'}
            self.sha_dict = {}
            self.para_sha_dict = {}
            self.chr_dict = {}
            self.temp_conv = self.temp_conv_qt64
            self.calc_hv = self.calc_hv_qt64
            self.reserved_stctstr = '<HB'
        elif self.protocol_version in [3.2, 3.4, 3.5]:
            reserved_key = 'reserved1_id'
            self.temp_dict = {0: 'Temp_web', 1: 'Temp_TMB_FPGA', 2: 'Temp_TMB_PMIC', 5: 'Temp_TX_2', 6: 'Temp_TX_3',
                              7: 'Temp_TX_4', 10: 'Temp_RX', 12: 'Temp_BMB_FPGA', 13: 'Temp_BuckBoost', 17: 'Temp_WCD'}
            self.volt_dict = {3: 'U_TMB_IN_12V', 24: 'U_TMB_2.5V', 25: 'U_TMB_5V', 26: 'U_TMB_3.3V', 8: 'TX_HV1',
                              9: 'TX_HV2',
                              11: 'U_Vbias_34V', 15: 'HV_BuckBoost', 16: 'U_IN'}
            self.sha_dict = {}
            self.para_sha_dict = {}
            self.chr_dict = {}
            self.temp_conv = self.transfer_value_at_qt128
            self.calc_hv = self.transfer_value_at_qt128
            self.reserved_stctstr = '<hB'
        elif self.protocol_version in [6.1]:
            reserved_key = 'reserved1_id'
            self.temp_dict = {3: 'Temp_CB_3', 4: 'Temp_CB_5', 5: 'TempA_RFB_1', 6: 'TempA_RFB_2',
                              7: 'TempA_TMB_1', 132: 'Temp_tmbl'}
            self.volt_dict = {133: 'HV_laser', 27: 'HV_apd', 134: 'HV_tp'}
            self.sha_dict = {36: 'SHA_1', 37: 'SHA_2', 38: 'SHA_3', 39: 'SHA_4'}
            self.para_sha_dict = {40: 'Para_SHA_1', 41: 'Para_SHA_2', 42: 'Para_SHA_3', 43: 'Para_SHA_4'}
            self.chr_dict = {}
            self.temp_conv = self.transfer_value_xt
            self.calc_hv = self.transfer_value_xt
            self.reserved_stctstr = '<hB'

        self.rs_all_keys = list(self.temp_dict.values()) + list(self.volt_dict.values()) + \
                           list(self.sha_dict.values()) + list(self.para_sha_dict.values()) + \
                           list(self.chr_dict.values())

        self.reserved_key = reserved_key[:-3]
        self.reserved_index = self.tail_index_dict[self.reserved_key][0]
        if online_flag:
            self.reserved_id_set = self.get_reserved_bit_info(key=reserved_key)
            print(len(self.reserved_id_set), self.reserved_id_set)
            if self.protocol_version in [1.4] and len(self.reserved_id_set) > 80:
                self.protocol_version = 1.45
                print('this is a P128 5.5 Lidar!')
                self.temp_dict = {0: 'Temp_RFB_1', 2: 'Temp_TMB_1', 66: 'TempD_CB_2', 67: 'TempD_CB_3', 4: 'TempA_CB_5',
                                  5: 'TempA_RFB_1', 7: 'TempA_TMB_1', 8: 'Temp_WCT', 9: 'Temp_TMB_FPGA',
                                  44: 'TempA_WPR', 53: "Temp_RBB_1", 65: 'TempD_LBB_1'}
            elif self.protocol_version in [3.2] and len(self.reserved_id_set) > 40:
                self.protocol_version = 3.22
                print('this is a QT128_B_sample Lidar!')
                reserved_key = 'reserved1_id'
                self.temp_dict = {0: 'Temp_TX_AVE', 1: 'Temp_TMB_PMIC', 2: 'Temp_TMB_FPGA', 4: 'Temp_TX_1',
                                  5: 'Temp_TX_2', 6: 'Temp_RX_2', 7: 'Temp_RX_AVE', 10: 'Temp_RX_1',
                                  12: 'Temp_BMB_FPGA', 13: 'Temp_BMB_BuckBoost', 17: 'Temp_WCD', 27: 'Temp_TMB_PD',
                                  39: 'Temp_TMB_FPGA_IN', 40: 'U_TMB_FPGA_VCCINT', 41: 'Temp_BMB_FPGA_IN'}
                self.volt_dict = {3: 'U_TMB_IN_12V', 24: 'U_TMB_2.5V', 25: 'U_TMB_5V', 26: 'U_TMB_3.3V',
                                  8: 'U_TXHV_16V',
                                  9: 'U_TXHV_5V', 11: 'U_Vbias_34V', 14: 'I_BMB_BuckBoost', 15: 'HV_BMB_BuckBoost',
                                  16: 'U_IN', 28: 'U_TMB_VREF_1V', 29: 'U_VTHR_HT', 30: 'U_VTHR_LT',
                                  31: 'U_TMB_VREF_2.5V',
                                  32: 'I_BMB_IN', 33: 'U_ASIL_3.3V', 34: 'U_BMB_VREF_2.5V', 35: 'U_BMB_T1_0.9V',
                                  36: 'U_BMB_VTTDDR_0.675V', 37: 'U_BMB_WCD_N19V', 38: 'U_BMB_3.3V',
                                  42: 'U_BMB_FPGA_VCCINT'}
                self.sha_dict = {}
                self.para_sha_dict = {}
                self.chr_dict = {}
                self.temp_conv = self.transfer_value_at_qt128
                self.calc_hv = self.transfer_value_at_qt128
                self.reserved_stctstr = '<hB'

    def init_hardware_port(self):
        self.relay = None
        self.power = None
        self.ampere = None
        self.motor = None
        self.ethernet_port = 'eno2'

    def get_correction_info(self, correction_file):
        '''
        read csv from input correction file, data with 'float64' type in np.array in radius
        :param correction_file:
        :return: self.azi_correction and self.ele_correction
        '''
        d_type = 'float64'
        df = pd.read_csv(correction_file)
        self.azi_correction = np.deg2rad(np.array(df, dtype=d_type)[:, 2])  # (128, )
        self.ele_correction = np.deg2rad(np.array(df, dtype=d_type)[:, 1])  # (128, )

    def get_firetime_info(self, firetime_file):
        '''
        read csv from input correction file, data with 'float64' type in np.array in radius
        :param firetime_file:
        :return: self.fire_time_base and self.ele_correction
        '''
        d_type = 'float64'
        df = pd.read_excel(firetime_file)
        self.spin_speed = self.get_spin_rate_cgi()
        self.get_actual_operation_mode()
        tmp_speed = self.spin_speed / 60 * 360 / 1e9

        self.fire_time = np.array(np.array(df)[2:, 1:] * tmp_speed, dtype=d_type)
        self.fire_time_base = np.deg2rad(self.fire_time[:, 0::2])  # (128, 8)

        if self.protocol_version in [1.3, 1.4, 1.45]:
            self.op_sidx = [0, 8, 8, 12][self.operation_mode]
            self.op_eidx = [8, 0, 12, 16][self.operation_mode]
            self.firetime_aft_op = np.deg2rad(self.fire_time[:, self.op_sidx:self.op_eidx])

    def get_actual_operation_mode(self):
        '''
        get current operation mode from 'high_t_shutdown' flag in udp tail
        udp version 1.4 P128 has 4 operation modes: 0 high resolution, 1: shut dowm 2 standard, 3. energy efficient mode
        :return: current operation mode in self.operation
        '''
        data, addr = self.udpsock.recvfrom(self.data_size + 42)
        tail_dict = self.parse_udp_tail_general(data, data_type='all', return_type='dict_wo_reserveds')
        self.operation_mode = tail_dict['high_t_shutdown']

    def input_lidar_fabrication_info(self, h, b):
        '''
        input lidar's fabrication information manually, unit in meter
        :param h: 雷达出光中心与雷达中心的距离 P128: 0.04m
        :param b: 雷达出光中心与雷达中心的距离 P128: 0.012m
        :return:
        '''
        self.b = b
        self.h = h

    def get_lidar_type(self):
        '''
        return lidar type in string for better comprehension
        :return:
        '''
        if self.protocol_version in [1.3, 1.4, 1.45]:
            if self.laser_num == 128:
                self.lidar_type = 'Pandar128'
            elif self.laser_num == 64:
                self.lidar_type = 'P64S'
            elif self.laser_num == 40:
                self.lidar_type = 'P40S'
            elif self.laser_num == 80:
                self.lidar_type = 'Pandar80'
            elif self.laser_num == 90:
                self.lidar_type = 'BD90'
            else:
                self.lidar_type = 'unknown'
        elif self.protocol_version in [64.6]:
            self.lidar_type = 'Pandar64'
        elif self.protocol_version in [99.99] and self.laser_num == 40:
            self.lidar_type = 'Pandar40'
        elif self.protocol_version in [3.1]:
            self.lidar_type = 'QT64'
        elif self.protocol_version in [3.2, 3.4, 3.5, 3.22]:
            self.lidar_type = 'QT128'
        elif self.protocol_version in [4.1, 4.3]:
            self.lidar_type = 'AT128'
        elif self.protocol_version in [6.1]:
            if self.laser_num == 32 and self.block_num == 8:
                self.lidar_type = 'XT-32'
            elif self.laser_num == 16 and self.block_num == 8:
                self.lidar_type = 'XT-16'
            elif self.laser_num == 32 and self.block_num == 6:
                self.lidar_type = 'XTM'
            else:
                self.lidar_type = 'unknown'
        else:
            self.lidar_type = 'unknown'

    def get_temp_range_reboot_time(self):
        """
        :return: 各种雷达高低温范围
        """
        if self.protocol_version in [1.3, 1.4, 1.45]:
            self.temp_range = [-40, 85]
            self.udp_time = 60
            self.reboot_time = 70
            self.upgrade_time = 130
        elif self.protocol_version in [64.6]:
            self.temp_range = [-20, 65]
            self.udp_time = 30
            self.reboot_time = 40
            self.upgrade_time = 280
        elif self.protocol_version in [99.99] and self.laser_num == 40:
            self.temp_range = [-20, 65]
            self.udp_time = 30
            self.reboot_time = 40
            self.upgrade_time = 400
        elif self.protocol_version in [3.1]:
            self.temp_range = [-20, 65]
            self.udp_time = 45
            self.reboot_time = 50
            self.upgrade_time = 200
        elif self.protocol_version in [3.2, 3.4, 3.5, 3.22]:
            self.temp_range = [-20, 65]
            self.udp_time = 10
            self.reboot_time = 30
            self.upgrade_time = 180
        elif self.protocol_version in [4.1, 4.3]:
            self.temp_range = [-40, 85]
            self.udp_time = 5
            self.reboot_time = 30
            self.upgrade_time = 150
        elif self.protocol_version in [6.1]:
            self.temp_range = [-20, 65]
            self.udp_time = 30
            self.reboot_time = 40
            self.upgrade_time = 180
        else:
            self.temp_range = [-20, 65]
            self.udp_time = 30
            self.reboot_time = 40
            self.upgrade_time = 250

    def gen_gps_socket(self):
        tmp = self.get_lidar_config_cgi(info_type='GpsPort')
        self.gps_port = int(tmp)
        self.gpssock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gpssock.bind(('', self.gps_port))
        self.gpssock.settimeout(1.05)
        gdata, addr = self.gpssock.recvfrom(700)
        self.gps_data_size = len(gdata)

    def get_reserved_bit_info(self, key='reserved1_id'):
        temp_index = self.tail_index_dict[key]

        self.reserved_loops = 500 + 500 * (self.protocol_version in [6.1])
        for _ in range(30):
            strsteam = b''
            count = 0
            for _ in range(self.reserved_loops):
                data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
                if len(data) == self.data_size:
                    tail_data = data[temp_index[0]:temp_index[0] + temp_index[1]]
                    strsteam = strsteam + tail_data
                    count += 1
            strctstr = '<' + 'B' * count
            id_set = list(set(struct.unpack(strctstr, strsteam)))
            if len(id_set) > 15:
                return id_set
        return []

    def read_pcap_ut_in_pieces_generator(self, file_handle, start_pos=0):
        """Lazy function (generator) to read a file piece by piece.
        upd: 11,  tcp:06;  24th bytes of 42
        """
        if start_pos:
            first_flag = 0
            file_handle.seek(start_pos, 0)
        else:
            first_flag = 1

        chunk_data = 16 + 42

        while True:
            if first_flag:
                udp_info = file_handle.read(82)
                data_type = udp_info[40 + 23]
                if data_type == 17:
                    first_data_size = struct.unpack('>H', udp_info[78:80])[0] - 8
                    first_data = file_handle.read(first_data_size)
                    if first_data[0:2] == b'\xee\xff':
                        print('first data is point cloud udp')
                        first_flag = 0
                        yield ('u', first_data)
                    elif first_data[0:2] == b'\xcd\xdc':
                        print('first data is safety udp')
                        first_flag = 0
                        yield ('u', first_data)
                    elif first_data[0:2] == b'\xff\xee':
                        print('first data is gps udp')
                        first_flag = 0
                        yield ('u', first_data)
                elif data_type == 6:
                    first_data_size = struct.unpack('>H', udp_info[56:58])[0] - 28
                    print(struct.unpack('>H', udp_info[56:58])[0])
                    first_data = udp_info[74:] + file_handle.read(first_data_size)
                    print('first data is tcp, data size is {}'.format(first_data_size + 8))
                    first_flag = 0
                    yield ('t', first_data)
            else:
                udp_info = file_handle.read(chunk_data)
                try:
                    data_type = udp_info[16 + 23]
                except (struct.error, IndexError):
                    print(len(udp_info))
                    return
                if data_type == 17:
                    # try:
                    data_size = struct.unpack('>H', udp_info[chunk_data - 4:chunk_data - 2])[0] - 8
                    data = file_handle.read(data_size)
                    # print(udp_info[16:40], udp_info[39], udp_info[32:34])
                    yield ('u', data)
                    # except struct.error:
                    #     print(len(udp_info))
                    #     return
                elif data_type == 6:
                    # try:
                    data_size = struct.unpack('>H', udp_info[32:34])[0] - 28
                    data = udp_info[chunk_data - 8:] + file_handle.read(data_size)
                    # print(udp_info[16:40], udp_info[39], udp_info[32:34])
                    yield ('t', data)

    def pick_out_first_udp(self, pcap_file):

        with open(pcap_file, 'rb') as file_handle:
            for data_type, data in self.read_pcap_ut_in_pieces_generator(file_handle):
                if data_type == 'u' and len(data) > 500:
                    cur_pos = file_handle.tell()
                    return data, cur_pos

    def parse_pcap_return_udp_in_list(self, pcap_file, udp_pkgs=None):
        '''
        parse pcap file and return udp_pkgs in list
        :param pcap_file: pcap file to parse
        :param udp_pkgs: should [start_idx, end_index], return udp list with k udp packages
        :return:
        '''
        ''' the method of send datas from pcap is:
        with open(pcap_file, 'rb') as file_handle:
            for data_type, data in self.read_pcap_ut_in_pieces_generator(file_handle):
                data_type = 'u' or 't'
        '''
        pcap2udp = []
        header1 = 40
        header2 = 42
        interval = 16
        with open(pcap_file, 'rb') as file_handle:
            data = file_handle.read()
        udp_num = int((len(data) - header1 + interval) / (self.data_size + header2 + interval))
        if udp_pkgs:
            udp_start, udp_end = udp_pkgs
            # loops = udp_pkgs + 1
        else:
            udp_start = 1
            udp_end = udp_num
        for i in range(udp_start, udp_end):
            idx = header1 + header2 * i + (interval + self.data_size) * (i - 1)
            udp_data = data[idx: idx + self.data_size]
            pcap2udp.append(udp_data)
        return pcap2udp

    def get_udp_num_psec(self, online_flag=1):
        udp_num_psec = int(72000 / self.block_num / ((self.laser_num < 100) + 1)) * (
                self.protocol_version in [1.3, 1.4, 1.45]) + 3000 * (self.protocol_version == 3.1) + 5000 * (
                               self.protocol_version == 6.1) + 6000 * (self.protocol_version == 64.6) + 9000 * (
                               self.protocol_version in [3.2, 3.22]) + 3600 * (self.protocol_version == 99.99) + \
                       12550 * (self.protocol_version in [4.1, 4.3]) + \
                       2250 * (self.protocol_version in [3.4, 3.5])
        if online_flag:
            fov_ratio = 1
            method_value, r = self.get_lidar_fov_cgi()
            if self.protocol_version in [4.1, 4.3]:
                total_fov = 120
            elif self.protocol_version in [64.4, 99.99]:
                total_fov = 360
            else:
                total_fov = 3600
            return_num = self.get_return_num()
            return_ratio = return_num / 2
            if r:
                if 'lidar_range' in r:
                    fov_ratio = (r['lidar_range'][1] - r['lidar_range'][0]) / total_fov
                else:
                    fov_ratio = (float(r['EndAngle']) - float(r['StartAngle'])) / total_fov
            self.udp_num_psec = int(min(fov_ratio, 1) * udp_num_psec)
            self.udp_num_psec_rm = int(self.udp_num_psec * return_ratio)
            if self.protocol_version in [1.4, 1.45] and self.laser_num == 90:
                self.resolution_mode = self.get_resolution_cgi()
                if self.resolution_mode == 'high' and return_num == 1:
                    self.udp_num_psec_rm = self.udp_num_psec_rm * 2
            elif self.protocol_version in [1.3, 1.4, 1.45]:
                self.resolution_mode = self.get_resolution_cgi()
            # print("debug: ", self.udp_num_psec, self.udp_num_psec_rm)
        else:
            return_ratio = self.return_num / 2
            self.udp_num_psec = udp_num_psec
            self.udp_num_psec_rm = int(self.udp_num_psec * return_ratio)
            # self.resolution_mode = 'high'

    def get_standard_resolution(self, online_flag=1):
        self.standard_spd = 600
        self.delta_timestamp = (1 / self.udp_num_psec) * 1e6
        if self.protocol_version in [1.3, 1.4, 1.45] and self.laser_num == 128:
            if self.resolution_mode == 'high':
                self.standard_resolution = 10
            else:
                self.standard_resolution = 20
            # self.delta_timestamp = 27.7
        elif self.protocol_version in [1.3, 1.4, 1.45] and self.laser_num in [64, 40]:
            self.standard_resolution = 20
            # self.delta_timestamp = 55.4
        elif self.protocol_version in [1.3, 1.4, 1.45] and self.laser_num in [80]:
            self.standard_resolution = 12
            # self.delta_timestamp = 66.4
        elif self.protocol_version in [1.4, 1.45] and self.laser_num in [90]:
            if self.resolution_mode == 'high':
                self.standard_resolution = 10
            else:
                self.standard_resolution = 20
            # self.delta_timestamp = 111.1
        elif self.protocol_version in [6.1]:
            self.standard_resolution = 18
            self.delta_timestamp = 49.9
        elif self.protocol_version in [3.1]:
            self.standard_resolution = 60
            # self.delta_timestamp = 166.7
        elif self.protocol_version in [3.2, 3.4, 3.5, 3.22]:
            self.standard_resolution = 40
            # self.delta_timestamp = 111.1
        elif self.protocol_version in [4.1, 4.3]:
            self.standard_resolution = 5
            self.delta_timestamp = 41.7
            self.standard_spd = 200
        elif self.protocol_version in [64.6]:
            self.standard_resolution = 20
            self.delta_timestamp = 166.7
        elif self.protocol_version in [99.99]:
            self.standard_resolution = 20
            self.delta_timestamp = 277.8
        else:
            self.standard_resolution = 20
        if online_flag:
            spin_spd = self.get_spin_rate_cgi()
            standard_resolution_sp = int((spin_spd / self.standard_spd) * self.standard_resolution)
            self.standard_resolution_sp = standard_resolution_sp
            return_num = self.get_return_num()
            tmp = self.get_rotate_direct_cgi()
            rot_dir = (tmp == 'clockwise') * 1 - (tmp == 'counterclockwise') * 1
            self.resolution_per_udp = np.ceil(standard_resolution_sp * self.block_num / return_num) * rot_dir
            self.resolution_per_azm = standard_resolution_sp * rot_dir
            # self.delta_timestamp_udp = self.delta_timestamp / return_num * 2
            # print(self.resolution_per_udp, self.resolution_per_azm, self.standard_resolution_sp)
            self.delta_timestamp_udp = (1 / self.udp_num_psec_rm) * 1e6
            if self.protocol_version in [4.3]:
                self.delta_timestamp_udp = self.delta_timestamp / return_num * 2
                self.timestamp_gap = 42950 / (spin_spd / 200)
                # self.timestamp_gap = 42250 / (spin_spd / 200)
            return standard_resolution_sp
        else:
            standard_resolution_sp = int((self.spin_spd / self.standard_spd) * self.standard_resolution)
            self.standard_resolution_sp = standard_resolution_sp
            rot_dir = 1
            self.resolution_per_udp = np.ceil(standard_resolution_sp * self.block_num / self.return_num) * rot_dir
            self.resolution_per_azm = standard_resolution_sp * rot_dir
            self.delta_timestamp_udp = (1 / self.udp_num_psec_rm) * 1e6
            if self.protocol_version in [4.3]:
                self.delta_timestamp_udp = self.delta_timestamp / self.return_num * 2
                self.timestamp_gap = 42950 / (self.spin_spd / 200)
            return standard_resolution_sp

    def get_rbt_tcpdump_time(self):
        return 19 + 25 * (self.protocol_version in [1.4, 1.45] and self.laser_num == 128)

    def ptc_sender(self, raw_cmd_code, payload, timeout=1):
        self.ptc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ptc_socket.settimeout(timeout)
        self.ptc_socket.connect((self.host, 9347))

        cmd_code = struct.pack('>B', raw_cmd_code)
        if not payload or payload.upper() == "NONE":
            data = b'\x47\x74' + cmd_code + b'\x00\x00\x00\x00\x00'
        else:
            payload_len = len(payload)
            data = b'\x47\x74' + cmd_code + b'\x00' + struct.pack('>L', payload_len) + payload
        try:
            self.ptc_socket.send(data)
        except socket.timeout as e1:
            print('ptc SEND timeout: command 0x{:02x} with {}'.format(raw_cmd_code, e1))
            return {'send_timeout': 1}
        try:
            response = self.ptc_socket.recv(8)
        except socket.timeout as e1:
            print('ptc RECV_1 timeout: command 0x{:02x} with {}'.format(raw_cmd_code, e1))
            return {'recv_timeout': 1}
        else:
            r_cmd = int.from_bytes(response[2:3], 'big')
            r_returnCode = int.from_bytes(response[3:4], 'big')
            r_length = int.from_bytes(response[4:8], 'big')
            if r_length:
                try:
                    response_payload = b''
                    while len(response_payload) < r_length:
                        response_payload += self.ptc_socket.recv(r_length)
                except socket.timeout as e1:
                    print('ptc RECV_2 timeout: command 0x{:02x} with {}'.format(raw_cmd_code, e1))
                    return {'recv_timeout': 1}
            else:
                response_payload = ''

            final_response = {
                "response_command": r_cmd,
                "response_return_code": r_returnCode,
                "response_payload_length": r_length,
                "response_payload": response_payload
            }
            self.ptc_socket.close()
            return final_response

    def get_lidar_status_ptc(self):
        if self.protocol_version in [1.3, 1.4, 1.45]:
            tmp_list = ['BMB_T1', 'BMB_T2', 'LEB_RT_L1', 'LEB_RT_L2', 'Receive_Board_RT_R', 'Receive_Board_RT2',
                        'Top_circuit_RT3', 'Top_circuit_RT4']
        elif self.protocol_version in [64.6, 99.99]:
            tmp_list = ['BMB_T1', 'BMB_T2', 'LEB_RT_L', 'LEB_RT_R', 'Receive_Board_RT2', 'Top_circuit_RT3',
                        'Top_circuit_RT4', 'Top_circuit_RT5']
        else:
            return None

        info_key = ['system_uptime', 'motor_speed'] + tmp_list + ['gps_pps_lock', 'gps_gprmc_status', 'startup_times',
                                                                  'total_operation_time', 'ptp_lock_status', 'humidity',
                                                                  'reserved']

        response = self.ptc_sender(0x09, None)
        bytes = response['response_payload']
        strctstr = '>IH' + 'i' * 8 + 'B' * 2 + 'I' * 2 + 'BIB'
        tmp = list(struct.unpack(strctstr, bytes))
        info_value = tmp[0:2] + [tmp * 0.01 for tmp in tmp[2:10]] + [['unlock', 'lock'][tmp[10]]] + \
                     [['unlock', 'lock'][tmp[11]]] + tmp[12:14] + [
                         ['free run', 'tracking', 'locked', 'frozen'][tmp[14]]] + [tmp[15] * 0.1] + [tmp[16]]
        infos = dict(zip(info_key, info_value))
        print(infos)
        return info_value, infos

    def read_UDP_data_once(self, new_udp=1):
        if new_udp:
            self.udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.udpsock.bind(('', self.port))
            except OSError as e1:
                print('端口已被占用，检查是否有其他程序占用端口！')
                command = "netstat -tunlp | grep 2368 | awk '{print $6}' | cut -f 1 -d '/'"
                print('占用端口的进程号如下：')
                os.system(command)
                print('尝试使用【kill -9 进程号】关闭进程！')
                return b''
            else:
                self.udpsock.settimeout(180)
                data = b''
                for _ in range(10):
                    data, addr = self.udpsock.recvfrom(1500)
                    if len(data) > 600:
                        break
                return data
        else:
            data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
            return data

    def receive_multi_udps(self, udp_num):
        datas = []
        idx = 0
        while idx < udp_num:
            data, addr = self.udpsock.recvfrom(self.data_size + 42)
            if len(data) == self.data_size:
                datas.append(data)
                idx += 1

        return datas

    def collect_udps_from_pcap(self, pcap_file, start_pos, udp_num):
        '''
        :param start_pos: index of pcap, for program begin to read, should be int and last index of one whole single udp
        :return: datas, udp datas in list
        '''
        datas = []
        idx = 0
        with open(pcap_file, 'rb') as file_handle:
            for data_type, data in self.read_pcap_ut_in_pieces_generator(file_handle, start_pos=start_pos):
                if len(data) == self.data_size:
                    datas.append(data)
                    idx += 1
                if idx >= udp_num:
                    cur_pos = file_handle.tell()
                    break

        return datas, cur_pos

    def create_UDP_socket(self, ipv='ipv4', dest_ipv6=None):
        if ipv == 'ipv4':
            ipv4_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ipv4_sock.bind(('', self.port))
            ipv4_sock.settimeout(180)
            self.udpsock = ipv4_sock
        elif ipv == 'ipv6':
            if dest_ipv6:
                ipv6_sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                ipv6_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                ipv6_sock.bind(('', self.port))
                ipv6_sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_LOOP, True)
                mreq = struct.pack("16s15s".encode('utf-8'), socket.inet_pton(socket.AF_INET6, dest_ipv6),
                                   (chr(0) * 16).encode('utf-8'))
                ipv6_sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)
                ipv6_sock.settimeout(180)
                self.udpsock = ipv6_sock
            else:
                print('wrong destination ipv6 input!')
                self.udpsock = None
        else:
            print('wrong ipv type input!')

    def parse_udp_body_general(self, data_all, data_type='all', info_type='dist'):
        if data_type == 'all':
            data_body = data_all[self.header_size:self.header_size + self.body_size]
        elif data_type == 'body':
            data_body = data_all
        else:
            print('wrong data type input!')
            return None

        azmth_list = []
        dist_list = []
        int_list = []
        p40_flag = self.protocol_version in [99.99]
        at43_flag = self.protocol_version in [4.3]
        self.block_bt = self.laser_num * self.pt_byte_num + 2 + 2 * p40_flag + at43_flag
        dist_idx = range(2 + at43_flag + 2 * p40_flag, self.block_bt, self.pt_byte_num)
        int_idx = range(4 + at43_flag + 2 * p40_flag, self.block_bt, self.pt_byte_num)
        if self.conf_flag:
            cnfd_list = []
            cnfd_idx = range(5 + at43_flag, self.block_bt, self.pt_byte_num)
        for block_idx in range(self.block_num):
            start_idx = block_idx * self.block_bt  # start index of each block
            if at43_flag:
                azmth1, azmth2 = struct.unpack('<HB', data_body[start_idx:start_idx + 3])
                azmth = azmth1 + azmth2 / 256
            else:
                azmth = struct.unpack('<H', data_body[start_idx + 2 * p40_flag:start_idx + 2 + 2 * p40_flag])[0]
            azmth_list.append(azmth)
            dist_list_i = [
                round((struct.unpack('<H', data_body[start_idx + di:start_idx + di + 2])[0]) * self.dist_unit, 2) for
                di in dist_idx]
            dist_list.append(dist_list_i)
            int_list_i = [struct.unpack('<B', data_body[start_idx + ii:start_idx + ii + 1])[0] for ii in int_idx]
            int_list.append(int_list_i)
            if self.conf_flag:
                cnfd_list_i = [struct.unpack('<B', data_body[start_idx + ci:start_idx + ci + 1])[0] for ci in cnfd_idx]
                cnfd_list.append(cnfd_list_i)

        azmth_keys = ["azmth{}".format(temp_i + 1) for temp_i in range(len(azmth_list))]
        if info_type == 'dist':
            # dist_keys = [info_type + "{}".format(temp_i + 1) for temp_i in range(len(dist_list))]
            dist_dict = dict(zip(azmth_keys, dist_list))
            return dist_dict
        elif info_type == 'intensity':
            # int_keys = [info_type + "{}".format(temp_i + 1) for temp_i in range(len(int_list))]
            int_dict = dict(zip(azmth_keys, int_list))
            return int_dict
        elif info_type == 'azmth':
            # azmth_keys = ["azmth{}".format(temp_i + 1) for temp_i in range(len(azmth_list))]
            azmth_dict = dict(zip(azmth_keys, azmth_list))
            return azmth_dict
        elif info_type == 'azmth_idx_dict':
            # azmth_keys = ["azmth{}".format(temp_i + 1) for temp_i in range(len(azmth_list))]
            azmth_idx = [(self.header_size + 2 * p40_flag + block_idx * self.block_bt, 2 + at43_flag) for block_idx in
                         range(self.block_num)]
            azmth_idx_dict = dict(zip(azmth_keys, azmth_idx))
            return azmth_idx_dict
        elif info_type == 'distance_idx_block1_tuple':
            block1_start_idx = self.header_size + 2 + at43_flag
            block1_end_idx = self.header_size + 2 + at43_flag + self.laser_num * self.pt_byte_num
            strctstr = '<' + ('HB' + 'B' * self.conf_flag) * self.laser_num
            interval = 2 + self.conf_flag
            return block1_start_idx, block1_end_idx, strctstr, interval
        elif info_type == 'blocks_info':
            block1_start_idx = self.header_size + 2 + at43_flag
            block_bytes = self.laser_num * self.pt_byte_num
            block_interval = 2 + at43_flag + self.laser_num * self.pt_byte_num
            channel_bytes = 3 + self.conf_flag
            return block1_start_idx, block_bytes, block_interval, channel_bytes
        elif info_type == 'cloudpoint_crc' and self.protocol_version in [1.4, 1.45]:
            return struct.unpack('<I', data_body[-4:])[0]
        elif info_type == 'confidence' and self.conf_flag:
            # conf_keys = [info_type + "{}".format(temp_i + 1) for temp_i in range(len(cnfd_list))]
            conf_dict = dict(zip(azmth_keys, cnfd_list))
            return conf_dict
        elif info_type == 'confidence' and self.conf_flag == 0:
            print("cloud points of this Lidar have no confidence")
            return None
        else:
            print("wrong info type")
            return None

    def parse_udp_tail_general(self, data_all, data_type='all', return_type="list"):
        if self.protocol_version in [1.3, 3.1, 4.1]:
            tail_lord = self.__parse_udp_tail_v13_qt__(data_all, return_type=return_type)
        elif self.protocol_version in [6.1]:
            tail_lord = self.__parse_udp_tail_xt__(data_all, return_type=return_type)
        elif self.protocol_version in [1.4, 1.45, 3.2, 3.4, 3.5, 3.22]:
            tail_lord = self.__parse_udp_tail_v14__(data_all, data_type=data_type, return_type=return_type)
        elif self.protocol_version in [4.3]:
            tail_lord = self.__parse_udp_tail_v43__(data_all, data_type=data_type, return_type=return_type)
        elif self.protocol_version in [64.6] or self.laser_num == 40:
            tail_lord = self.__parse_udp_tail_unnmb__(data_all, return_type=return_type)
        else:
            print("this type udp version can not be parsed temporarily")
            tail_lord = None
        return tail_lord

    def parse_udp_all_qt34_35(self, data):
        if self.protocol_version not in [3.4, 3.5]:
            print('this function not supports non-QT128 lidar type!!')
            return None, None, None
        else:
            tmp_ratio = 3 - data[10]
            body_data = data[self.header_size:self.header_size + self.body_size - 4]
            safety_data = data[self.header_size + self.body_size:self.header_size + self.body_size + 5]
            tail_data = data[self.header_size + self.body_size + 17:self.header_size + self.body_size + 47]
            data_frag = body_data + safety_data + tail_data
            # print(len(body_data), len(safety_data), len(tail_data), len(data_frag))
            stctstr = '<' + ('H' + ('HB' + 'B' * self.conf_flag) * self.laser_num) * self.block_num + \
                      'BBBH' + 'hBHBhBHBBHBBBBBBIBI'
            blk_count = (1 + (2 + self.conf_flag) * self.laser_num)  # * self.block_num
            tmp = struct.unpack(stctstr, data_frag)

            safety_keys = ['FS_version', 'lidar_state', 'code_type', 'rolling_counter', 'total_code_num',
                           'fault_code_id', 'fault_code']
            FS_version, raw_bits1, raw_bits2, flt_code = tmp[blk_count * self.block_num: blk_count * self.block_num + 4]
            lidar_state = raw_bits1 >> 5
            code_type = (raw_bits1 >> 3) % 4  # 1: current; 2: history
            roll_cnt = raw_bits1 % 8
            total_cd_num = raw_bits2 >> 4
            flt_cd_id = raw_bits2 % 16
            safety_values = [FS_version, lidar_state, code_type, roll_cnt, total_cd_num, flt_cd_id, flt_code]
            safety_dict = dict(zip(safety_keys, safety_values))

            tail_keys = ["reserved1", "reserved1_id", "reserved2", "mode_flag", "reserved2", "reserved2_id",
                         "azimuth_flag", "high_t_shutdown", "return_mode", "motor_speed", "utc_time", "pcap_timestamp",
                         "factory_info", "udp_sequence"]
            tail_values = list(tmp[-19:-9]) + [self.transfer_time_list(list(tmp[-9:-3]))] + list(tmp[-3:])
            tail_dict = dict(zip(tail_keys, tail_values))

            ofs = int(tail_dict['mode_flag'])
            body_dict = {}
            for blk_idx in range(self.block_num):
                raw_distances = tmp[1 + blk_count * blk_idx: blk_count * (blk_idx + 1): (2 + self.conf_flag)]
                distances_i = [round(dis * self.dist_unit, 4) for dis in raw_distances]
                intensitys_i = list(tmp[2 + blk_count * blk_idx: blk_count * (blk_idx + 1): (2 + self.conf_flag)])
                laserid_i = self.laser_id_list[blk_idx % int(4 / tmp_ratio) // int(2 / tmp_ratio) + ofs]
                body_dict["azmth{}".format(blk_idx + 1)] = (
                    tmp[blk_count * blk_idx], laserid_i, distances_i, intensitys_i)
            return body_dict, safety_dict, tail_dict

    def parse_safety_udp_data_general(self, data, info_type='list'):
        key_list = ['FS_version', 'safety_time', 'lidar_operation_state', 'fault_state', 'fault_code_type',
                    'rolling_counter', 'total_fault_code_number', 'fault_code_id', 'fault_code', 'TDM_id',
                    'tdm_value', 'crc']
        if info_type == 'key_list':
            return key_list

        if self.protocol_version in [4.3]:
            if data[0:2] == b'\xcd\xdc':
                safety_time = self.transfer_lidar_time_bytes(data[3:13])
                # print(data[3:13], safety_time)
                data_frag = data[2:3] + data[13:19] + data[23:26] + data[50:54]
                strctstr = '<BBBBBBBBHI'
                FS_version, lidar_operation_state, fault_state, fault_code_type, rolling_counter, \
                total_fault_code_number, fault_code_id, TDM_id, tdm_value, crc \
                    = struct.unpack(strctstr, data_frag)
                fault_code = binascii.b2a_hex(data[19:23]).decode()
                value_list = [FS_version, safety_time, lidar_operation_state, fault_state, fault_code_type,
                              rolling_counter, total_fault_code_number, fault_code_id, fault_code, TDM_id,
                              tdm_value, crc]
                safety_dict = dict(zip(key_list, value_list))
                if info_type == 'dict':
                    return safety_dict
                elif info_type == 'list':
                    return value_list
                else:
                    print("wrong info_type input!")
                    return [np.nan]
            else:
                return [len(data)]
        else:
            print("the safety udp data of this type not supported")
            return [np.nan]

    def parse_gps_udp(self, data, info_type='dict', replace_flag=0):
        key_list = ['ymdhms', 'us', 'format', 'gps_status', 'pps_status']
        date_data = data[2:8] + data[12:14] + data[10:12] + data[8:10]
        us_data = data[14:18]
        format_data = data[18:102]
        status_data = data[506:508]
        # print(data[:14], date_data)
        tmp = str(date_data, encoding='utf-8')
        date_info = ''.join(list(itertools.chain.from_iterable(zip(tmp[1::2], tmp[::2]))))
        us_value, gps_status, pps_status = struct.unpack('<IBB', us_data + status_data)
        if replace_flag:
            format_data = format_data.replace(b'\x00', b'\x30')
        # print(format_data)
        format_info = str(format_data, encoding='utf-8')  # 'ISO-8859-1') # utf-8')
        value_list = [date_info, us_value, format_info, hex(gps_status), pps_status]
        # print(value_list)
        if info_type == 'dict':
            info_dict = dict(zip(key_list, value_list))
            return info_dict
        elif info_type == 'list':
            return value_list
        else:
            print("wrong info type input")
            return None

    def parse_sha_in_udp(self):
        reserved_key = 'reserved1_id'
        if self.protocol_version in [1.3, 1.4, 1.45]:
            self.sha_dict = {36: 'SHA_1', 37: 'SHA_2', 38: 'SHA_3', 39: 'SHA_4'}
            self.para_sha_dict = {40: 'Para_SHA_1', 41: 'Para_SHA_2', 42: 'Para_SHA_3', 43: 'Para_SHA_4'}
        elif self.protocol_version in [64.6, 99.99]:
            self.sha_dict = {12: 'SHA_1', 13: 'SHA_2', 14: 'SHA_3', 15: 'SHA_4'}
            self.para_sha_dict = {16: 'Para_SHA_1', 17: 'Para_SHA_2', 18: 'Para_SHA_3', 19: 'Para_SHA_4'}
        elif self.protocol_version in [4.1, 4.3]:
            # self.sha_dict = {37: 'SHA_1', 36: 'SHA_2', 39: 'SHA_3', 38: 'SHA_4'}
            # self.para_sha_dict = {41: 'Para_SHA_1', 40: 'Para_SHA_2', 43: 'Para_SHA_3', 42: 'Para_SHA_4'}
            # self.sha_dict = {49: 'SHA_1', 48: 'SHA_2', 51: 'SHA_3', 50: 'SHA_4'}
            # self.para_sha_dict = {53: 'Para_SHA_1', 52: 'Para_SHA_2', 55: 'Para_SHA_3', 54: 'Para_SHA_4'}
            self.sha_dict = {51: 'SHA_1', 50: 'SHA_2', 49: 'SHA_3', 48: 'SHA_4'}
            self.para_sha_dict = {55: 'Para_SHA_1', 54: 'Para_SHA_2', 53: 'Para_SHA_3', 52: 'Para_SHA_4'}
            reserved_key = 'reserved2_id'
        elif self.protocol_version in [6.1]:
            self.sha_dict = {128: 'SHA_1', 129: 'SHA_2', 130: 'SHA_3', 131: 'SHA_4'}
            self.para_sha_dict = {40: 'Para_SHA_1', 41: 'Para_SHA_2', 42: 'Para_SHA_3', 43: 'Para_SHA_4'}
        else:
            self.sha = None
            self.para_sha = None
            return None, None
        self.get_reserved_bit_info(key=reserved_key)
        self.reserved_key = reserved_key[:-3]
        self.reserved_index = self.tail_index_dict[self.reserved_key][0]
        for _ in range(30):
            reserved_data_list = []
            for _ in range(self.reserved_loops):
                data, addr = self.udpsock.recvfrom(self.data_size + 42)
                if len(data) == self.data_size:
                    reserved_data_list.append(data[self.reserved_index: self.reserved_index + 3])
            reserved_pair = list(map(self.parse_sha, reserved_data_list))
            reserved_id = [i for i, j in reserved_pair]
            sha = ''
            para_sha = ''
            try:
                for x in self.sha_dict.values():
                    sha += reserved_pair[reserved_id.index(x)][1]
                self.sha = sha
                for x in self.para_sha_dict.values():
                    para_sha += reserved_pair[reserved_id.index(x)][1]
                self.para_sha = para_sha
                return sha, para_sha
            except ValueError:
                print('waiting for reserved bits parsing')
                continue

    def parse_reserved_bits_onerow(self):
        reserved_data_list = []
        for _ in range(15):
            for _ in range(self.reserved_loops):
                data, addr = self.udpsock.recvfrom(self.data_size + 42)
                if len(data) == self.data_size:
                    reserved_data_list.append(data[self.reserved_index: self.reserved_index + 3])

            reserved_pair = list(map(self.parse_reserved_bits, reserved_data_list))
            # print(reserved_pair)
            # print([(i, j) for i, j in reserved_pair if i != 'HV_tp'])
            reserved_id = [i for i, j in reserved_pair]
            reserved_dict = {}
            reserved_dict.fromkeys(self.rs_all_keys)
            try:
                for key in self.rs_all_keys:
                    reserved_dict[key] = reserved_pair[reserved_id.index(key)][1]
                break
            except ValueError:
                continue

        return reserved_dict

    def parse_reserved_bits(self, reserved_bit_bytes):
        reserved_bit = struct.unpack(self.reserved_stctstr, reserved_bit_bytes)
        id = reserved_bit[1]
        value = reserved_bit[0]

        if id in self.temp_dict.keys():
            id_des = self.temp_dict[id]
            out_value = self.temp_conv(value)
        elif self.protocol_version in [3.1] and id in self.volt_dict.keys():
            qt64_mapping = {5: 11, 7: 6, 8: 31, 11: 21}
            id_des = self.volt_dict[id]
            out_value = self.calc_hv(value) * qt64_mapping[id]
        elif id in self.volt_dict.keys():
            id_des = self.volt_dict[id]
            out_value = self.calc_hv(value)
        elif id in self.sha_dict.keys():
            id_des = self.sha_dict[id]
            try:
                out_value = chr(value & 0x00ff) + chr(value >> 8)
            except ValueError:
                out_value = ''
        elif id in self.para_sha_dict.keys():
            id_des = self.para_sha_dict[id]
            try:
                out_value = chr(value & 0x00ff) + chr(value >> 8)
            except ValueError:
                out_value = ''
        elif id in self.chr_dict.keys():
            id_des = self.chr_dict[id]
            try:
                out_value = chr(value & 0x00ff) + chr(value >> 8)
            except ValueError:
                out_value = ''
        else:
            id_des = id
            out_value = value
        return [id_des, out_value]

    def transfer_utc_time_bytes(self, data_frag):
        x = list(struct.unpack('<BBBBBB', data_frag)) + [0, 0, 0]
        if x[0] < 100:
            x[0] = x[0] + self.utm_year_offset
        else:
            x[0] = x[0] + self.utm_year_offset - 70
        return time.mktime(time.struct_time(x))

    def transfer_lidar_time_bytes(self, data_frag):
        tmp = list(struct.unpack('>BBBBBBI', data_frag))
        # safety of AT should be 'big'
        x = tmp[0:6] + [0, 0, 0]
        if x[0] < 100:
            x[0] = x[0] + self.utm_year_offset
        else:
            x[0] = x[0] + self.utm_year_offset - 70
        timestamp = tmp[6]
        total_time = time.mktime(time.struct_time(x)) + timestamp / 1e6
        return total_time

    def transfer_time_list(self, x):
        y = x[:]
        if x[0] < 100:
            y[0] = x[0] + self.utm_year_offset
        else:
            y[0] = x[0] + self.utm_year_offset - 70
        y.extend([0, 0, 0])
        return time.mktime(time.struct_time(y))

    def parse_sha(self, reserved_bit_bytes):
        if self.protocol_version in [4.1, 4.3]:
            stctstr = '>HB'
        else:
            stctstr = '<HB'
        # stctstr = '<HB'
        reserved_bit = struct.unpack(stctstr, reserved_bit_bytes)
        tmp_sha_dict = self.sha_dict.copy()
        tmp_sha_dict.update(self.para_sha_dict)
        id = reserved_bit[1]
        value = reserved_bit[0]
        if id in tmp_sha_dict.keys():
            id_des = tmp_sha_dict[id]
            out_value = chr(value & 0x00ff) + chr(value >> 8)
        else:
            id_des = id
            out_value = value
        return [id_des, out_value]

    def temp_conv_p64(self, raw_data_in):
        raw_data_in = min(max(raw_data_in, 1), 4095)
        R2 = 1e4 * raw_data_in / (4096 - raw_data_in)

        T1 = 298.15
        T2 = 1.0 / (1.0 * np.log(R2 / 1e4) / 4100 + 1.0 / T1) - 273.15
        return round(T2, 2)

    def temp_conv_qt64(self, raw_data_in):
        T0 = 298.15
        B = 3375
        T1 = 1 / (np.log(5 * raw_data_in / (3.3 * 4096 - 5 * raw_data_in)) / B + 1 / T0)
        return round((T1 - 273.15), 2)

    def calc_hv_p64(self, value):
        adc_value = value
        power = 3.321
        R1 = 30100
        R2 = 3000000
        adc_voltage = 1.0 * (adc_value - 0.5) * power / 4096
        hv_voltage = adc_voltage - (R2 * (power - adc_voltage) / R1)
        return round(hv_voltage, 2)

    def calc_hv_qt64(self, value):
        return round((value / 4096) * 5, 2)

    def temp_conv_v45(self, raw_value):
        if raw_value > 2 ** 15:
            raw_value = raw_value - 2 ** 16
        return raw_value * 0.1

    # def calc_hv_v45(self, raw_value):
    #     vol_value = (raw_value - 65536) * 0.1
    #     return vol_value

    def calc_hv_v45(self, raw_value):
        return raw_value * 0.1

    def transfer_value_at_qt128(self, raw_value):
        return round(raw_value * 0.01, 2)

    def transfer_value_xt(self, raw_value):
        return round(raw_value * 0.1, 2)

    def login_p128_release_cgi(self, username, passwd):
        if self.protocol_version in [1.4, 1.45, 6.1] and self.cs_flag:
            url = self.base_url + 'action=get&object=login'
            bytes_passwd = passwd.encode("utf-8")
            passwdB64 = base64.b64encode(bytes_passwd)
            passwdB64_utf8 = str(passwdB64, encoding="utf8")
            data = {'key': username, 'value': passwdB64_utf8}
            res = requests.post(url, json=data, verify=False)
            data = json.loads(res.text)
            if 'Success' not in data['Head']['Message']:
                print(data)
            name = data.get('Body').get('cookie').get('name')
            uuid = data.get('Body').get('cookie').get('value')
            self.cookies = {name: uuid}
        else:
            self.cookies = None

    def connectWeb_except_error_page(self, driver, t):
        # writer: liyuda
        counter = time.time()
        while time.time() - counter < t:
            try:
                driver.get(self.url)
                break
            except WebDriverException as WDE:
                # print(WDE.msg)
                if "error page" in WDE.msg:
                    print(WDE.msg.split(':')[0])
                    pass

    def set_reset_cgi(self):
        set_p = {
            "action": "set",
            "object": "reset"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)

        # print(r)

    def set_reboot_cgi(self):
        set_p = {
            "action": "set",
            "object": "reboot"
        }
        try:
            self.login_p128_release_cgi(r'admin', r'123456')
        except AttributeError:
            # print("no login requested, cs_flag is {}".format(self.cs_flag))
            pass

        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        if r['Head']['ErrorCode'] == '0':
            print("cgi reboot starts, responce is {}".format(r))
        else:
            print("cgi reboot failed")

    def set_spin_rate_cgi(self, spin_spd=600):
        if spin_spd == 600:
            spd_value = "2"
        elif spin_spd == 1200:
            spd_value = "3"
        elif spin_spd == 300:
            spd_value = "1"
        elif self.protocol_version in [1.4, 1.45] and spin_spd == 900:
            spd_value = "5"
        elif self.protocol_version in [4.1, 4.3] and spin_spd == 150:
            spd_value = "5"
        elif self.protocol_version in [4.1, 4.3] and spin_spd == 200:
            spd_value = "4"
        elif self.protocol_version in [4.1, 4.3] and spin_spd == 750:
            spd_value = "6"
        elif self.protocol_version in [4.1, 4.3] and spin_spd == 400:
            spd_value = "7"
        elif self.protocol_version in [4.1, 4.3] and spin_spd == 500:
            spd_value = "8"
        else:
            print("input spinning speed not defined, set spinning speed as 600rpm")

        set_p = {
            "action": "set",
            "object": "lidar",
            "key": "spin_speed",
            "value": spd_value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, timeout=3, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p, timeout=3)
        r = json.loads(response.text)
        # print('This is response for set motor speed: {}.'.format(r))
        if r['Head']['ErrorCode'] == '0':
            spin_spd = self.get_spin_rate_cgi()
            print("spinning speed setting successful, current spinning speed is {}".format(spin_spd))
            standard_resolution_sp = int((spin_spd / self.standard_spd) * self.standard_resolution)
            self.standard_resolution_sp = standard_resolution_sp
            self.get_udp_num_psec()
            self.get_standard_resolution()
        else:
            print("regarding unknown error, setting spinning speed failed!")
            spin_spd = self.get_spin_rate_cgi()
            print("current spinning speed is {}".format(spin_spd))

    def set_trigger_method_cgi(self, method='angle_based'):
        value = '0'
        if method == 'angle_based':
            value = '0'
        elif method == 'time_based':
            value = '1'
        else:
            print("input trigger method not defined, set trigger method as angle_based")

        set_p = {
            "action": "set",
            "object": "lidar",
            "key": "trigger_method",
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        print('This is response for set trigger_method: {}.'.format(r))

    def set_destination_ip_udp_port_gps_cgi(self, port, destination_ip_str="255.255.255.255", GpsPort=10110):
        value = '{"IPv4":"' + destination_ip_str + '","Port":' + str(port) + ',"GpsPort":' + str(GpsPort) + "}'"
        # print(value)
        set_p = {
            "action": "set",
            "object": "lidar",
            "key": "ip_disnation",
            "value": value
        }
        if self.protocol_version in [4.3]:
            set_p = {
                "action": "set",
                "object": "lidar",
                "key": "ip_destination",
                "value": value
            }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        print(r)
        if r['Head']['ErrorCode'] == '0':
            self.port = port
            print("udp port is {}, destination ip is {}, GpsPort is {}".format(self.port, destination_ip_str, GpsPort))
        else:
            print("udp port stays unchanged:", self.port)

    def set_lidar_ip_udp_mask_cgi(self, lidar_ip_str):
        value = '{"DHCP":0,"IPv4":"' + lidar_ip_str + '","Mask":"255.255.255.0","Gateway":"192.168.1.1","VlanFlag":0,"VlanID":0}'
        # print(value)
        set_p = {
            "action": "set",
            "object": "control_port",
            "key": "ip",
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        try:
            if self.cs_flag and self.cookies:
                # response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
                response = requests.get(self.base_u, set_p, timeout=3, cookies=self.cookies, verify=False)
            else:
                # response = requests.get(self.base_url, set_p)
                response = requests.get(self.base_url, set_p, timeout=3)
            r = json.loads(response.text)
            print(r)
            print('This is response for setting lidar ip: {}.'.format(r))
            print("udp port stays unchanged:", self.host)
        except requests.exceptions.ReadTimeout:
            print("lidar ip has been changed to: ", lidar_ip_str)
            self.host = lidar_ip_str
            self.__set_cs_flag__()

    def get_trigger_method_cgi(self):
        get_p = {
            "action": "get",
            "object": "device_info"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        trigger_method = ['angle_based', 'time_based'][int(r["Body"]["Trigger_Method"])]
        return trigger_method

    def set_noise_filter_cgi(self, on_off='off'):
        value = '0'
        if on_off == 'off':
            value = '0'
        elif on_off == 'on':
            value = '1'
        else:
            print("input trigger method not defined, set noise filter as off")

        set_p = {
            "action": "set",
            "object": "lidar_data",
            "key": "noise_filtring",
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        print('This is response for set noise filter: {}.'.format(r))

    def get_noise_filter_cgi(self):
        get_p = {
            "action": "get",
            "object": "lidar_config"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        noise_filter = r['Body']['NoiseFiltering']
        if noise_filter == '0':
            res = 'off'
        else:
            res = 'on'
        return res

    def set_Retro_Multi_Reflection_cgi(self, on_off='off'):
        if not self.get_Retro_Multi_Reflection_cgi():
            print('retro multireflection is not supported!')
            return
        value = '0'
        if on_off == 'off':
            value = '0'
        elif on_off == 'on':
            value = '1'
        else:
            print("retro multi reflection not defined, set retro multi reflection as off")

        set_p = {
            "action": "set",
            "object": "lidar_data",
            "key": "retro_multireflection",
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        print('This is response for set retro multireflection: {}.'.format(r))

    def get_Retro_Multi_Reflection_cgi(self):
        get_p = {
            "action": "get",
            "object": "lidar_config"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        if 'RetroMultiReflection' in r['Body']:
            interstitial_filter = r['Body']['RetroMultiReflection']
            if interstitial_filter == '0':
                res = 'off'
            else:
                res = 'on'
            return res
        else:
            return None

    def set_blockage_detection_cgi(self, on_off='off'):
        if on_off == 'off':
            value = '0'
        elif on_off == 'on':
            value = '1'
        else:
            print("retro multi reflection not defined, set retro multi reflection as off")

        set_p = {
            "action": "set",
            "object": "blockage_detection",
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        if r['Head']['ErrorCode'] == '0':
            print('setting to {} successfully'.format(on_off))
        else:
            print('setting failed, response for set blockage detection: {}.'.format(r))

    def get_blockage_detection_cgi(self):
        # http: // 192.168.1.201 / pandar.cgi?action = get & object = blockage_detection
        # {"Head": {"ErrorCode": "0", "Message": "Success"}, "Body": {"blockage_detection": "0"}}
        get_p = {
            "action": "get",
            "object": "blockage_detection"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        if r['Head']['ErrorCode'] == '0':
            if 'Body' in r.keys():
                blockage_detection = r['Body']['blockage_detection']
                if blockage_detection == '0':
                    res = 'off'
                else:
                    res = 'on'
                return res
            else:
                return None
        else:
            print(r)
            print("unknown Error, blockage_detection getting failed!")
            return None

    def set_interstitial_points_cgi(self, on_off='off'):
        if not self.get_interstitial_points_cgi():
            print('interstitial filter is not supported!')
            return
        value = '0'
        if on_off == 'off':
            value = '0'
        elif on_off == 'on':
            value = '1'
        else:
            print("interstitial filter not defined, set interstitial filter as off")

        set_p = {
            "action": "set",
            "object": "lidar_data",
            "key": "interstitial_points",
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        print('This is response for set interstitial filter: {}.'.format(r))

    def set_standby_cgi(self, mode='standby', info_type='default'):
        '''
        :param mode: standby or operation
        :return:
        '''
        if mode == 'standby':
            value = '1'
        elif mode == 'operation':
            value = '0'
        else:
            print('wrong input!')
            return None
        set_p = {
            "action": "set",
            "object": "lidar_data",
            "key": "standbymode",
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, timeout=2, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p, timeout=2)
        r = json.loads(response.text)
        # print(r)
        if info_type == 'response_info':
            response_time = response.elapsed.total_seconds()
            status_code = response.status_code
            if r['Head']['ErrorCode'] == '0':
                standby_mode = mode
            else:
                print(r)
                standby_mode = 'unknown'
            return [response_time, status_code, standby_mode]

        if r['Head']['ErrorCode'] == '0':
            if 'Body' in r.keys():
                return r['Body']['standbymode']
            else:
                return None
        else:
            print("unknown Error, return mode setting failed!")
            return None

    def get_standby_cgi(self, info_type='standby_mode'):
        get_p = {
            "action": "get",
            "object": "lidar_data",
            "key": "standbymode"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if info_type == 'response_time':
            return response.elapsed.total_seconds()
        elif info_type == 'response_info':
            response_time = response.elapsed.total_seconds()
            status_code = response.status_code
            if r['Head']['ErrorCode'] == '0' and 'Body' in r.keys():
                tmp = r['Body']['standbymode']
                standby_mode = 'operation' * (tmp == '0') + 'standby' * (tmp == '1')
            else:
                print(r)
                standby_mode = 'unknown'
            return [response_time, status_code, standby_mode]
        else:
            if r['Head']['ErrorCode'] == '0':
                return r['Body']['standbymode']
            else:
                print("unknown Error, return mode setting failed!")
                return None

    def get_interstitial_points_cgi(self):
        get_p = {
            "action": "get",
            "object": "lidar_config"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if 'InterstitialPoints' in r['Body']:
            interstitial_filter = r['Body']['InterstitialPoints']
            if interstitial_filter == '0':
                res = 'off'
            else:
                res = 'on'
            return res
        else:
            return None

    def set_reflectivity_mapping_cgi(self, on_off='off'):
        if not self.get_reflectivity_mapping_cgi():
            print('Reflectivity Mapping is not supported!')
            return
        if on_off == 'off':
            value = '0'
        elif on_off == 'on':
            value = '1'
        elif on_off == 'on1':
            value = '1'
        elif on_off == 'on2':
            value = '2'
        else:
            value = '0'
            print("input Reflectivity Mapping not defined, set Reflectivity Mapping as off")

        set_p = {
            "action": "set",
            "object": "lidar_data",
            "key": "ReflectivityMapping",
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        print('This is response for set Reflectivity Mapping: {}.'.format(r))
        return

    def get_reflectivity_mapping_cgi(self):
        get_p = {
            "action": "get",
            "object": "lidar_config"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if 'ReflectivityMapping' in r['Body']:
            if r['Body']['ReflectivityMapping'] == '0':
                res = 'off'
            elif r['Body']['ReflectivityMapping'] == '1':
                res = 'on1'
            elif r['Body']['ReflectivityMapping'] == '2':
                res = 'on2'
            else:
                res = 'off'
            return res
        else:
            return None

    def set_return_mode_cgi(self, return_mode='last_return'):
        mode_value_mapping = {'last_return': '0', 'strongest_return': '1', 'last_and_strongest_return': '2',
                              'first_return': '3', 'last_and_first_return': '4', 'strongest_and_first_return': '5',
                              'dual_return': '2', 'triple_return': '6'}
        set_p = {
            "action": "set",
            "object": "lidar_data",
            "key": "lidar_mode",
            "value": mode_value_mapping[return_mode]
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        if r['Head']['ErrorCode'] == '2':
            return_mode_value, return_mode, dual_flag = self.get_return_mode_cgi()
            print("This return mode is not supported in this lidar type, current return mode is {}".format(return_mode))
        elif r['Head']['ErrorCode'] == '0':
            return_mode_value, return_mode, dual_flag = self.get_return_mode_cgi()
            print("return mode setting successful, current return mode is {}".format(return_mode))
            self.get_udp_num_psec()
            self.get_standard_resolution()
        else:
            return_mode_value, return_mode, dual_flag = self.get_return_mode_cgi()
            print("unknown Error, return mode setting failed!")
        return r['Head']['ErrorCode']

    def get_return_mode_cgi(self, info_type='return_mode'):
        mode_value_mapping = {'last_return': '0', 'strongest_return': '1', 'last_and_strongest_return': '2',
                              'first_return': '3', 'last_and_first_return': '4', 'strongest_and_first_return': '5',
                              'triple_return': '6'}
        value_mode_mapping = dict(zip(list(mode_value_mapping.values()), list(mode_value_mapping.keys())))
        get_p = {
            "action": "get",
            "object": "lidar_data",
            "key": "lidar_mode"
        }
        if self.protocol_version in [4.1, 4.3, 3.1]:
            value_mode_mapping['2'] = 'dual_return'
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        return_mode_value = r["Body"]["lidar_mode"]
        return_mode = value_mode_mapping[return_mode_value]
        dual_flag = return_mode_value not in ['0', '1', '3']

        if info_type == 'response_time':
            return response.elapsed.total_seconds()
        else:
            if r['Head']['ErrorCode'] == '0':
                return return_mode_value, return_mode, dual_flag
            else:
                return None, None, None

    def get_return_num(self):
        return_mode_value, return_mode, dual_flag = self.get_return_mode_cgi()
        if return_mode_value in ['0', '1', '3']:
            return 1
        elif return_mode_value in ['2', '4', '5']:
            return 2
        elif return_mode_value in ['6']:
            return 3
        else:
            print("return number unknown")
            return None

    def set_resolution_cgi(self, resolution_mode='high'):
        if self.protocol_version in [1.3, 1.4, 1.45]:
            standard_mode_value = '{"mode":' + str(0) + "}'"
            high_mode_value = '{"mode":' + str(1) + "}'"
            mode_value_mapping = {'standard': standard_mode_value, 'high': high_mode_value}
            set_p = {
                "action": "set",
                "object": "laser_control",
                "key": "high_resolution",
                "value": mode_value_mapping[resolution_mode]
            }
            self.login_p128_release_cgi(r'admin', r'123456')
            if self.cs_flag and self.cookies:
                response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
            else:
                response = requests.get(self.base_url, set_p)
            r = json.loads(response.text)
            if r['Head']['ErrorCode'] == '2':
                resolution_mode = self.get_resolution_cgi()
                print("This resolution is not supported in this lidar type, current resolution is {}".format(
                    resolution_mode))
            elif r['Head']['ErrorCode'] == '0':
                resolution_mode = self.get_resolution_cgi()
                print("resolution setting successful, current resolution is {}".format(resolution_mode))
            else:
                print("unknown response! response is {}".format(r))
        else:
            print("not supported in this lidar type!")

    def get_resolution_cgi(self, info_type='resolution'):
        if self.protocol_version in [1.3, 1.4, 1.45]:
            value_mode_mapping = {'0': 'standard', '1': 'high'}
            get_p = {
                "action": "get",
                "object": "high_resolution"
            }
            self.login_p128_release_cgi(r'admin', r'123456')
            if self.cs_flag and self.cookies:
                response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
            else:
                response = requests.get(self.base_url, get_p, timeout=1)
            r = json.loads(response.text)

            if info_type == 'response_time':
                return response.elapsed.total_seconds()
            elif r['Head']['ErrorCode'] == '0':
                resolution_mode = value_mode_mapping[r['Body']['mode']]
                return resolution_mode
            else:
                print("unknown resolution mode! response is {}".format(r))
                return None
        else:
            print("not supported in this lidar type!")
            return None

    def set_sync_angle_cgi(self, enable_flag=1, sync_angle=0):
        if sync_angle > 360 or sync_angle < 0:
            print("wrong synchronization angle")

        syn_value = '{"sync":' + str(enable_flag) + ',"syncAngle":' + str(sync_angle) + "}'"
        set_p = {
            "action": "set",
            "object": "lidar_sync",
            "key": "sync_angle",
            "value": syn_value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, timeout=3, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p, timeout=3)
        r = json.loads(response.text)
        print('This is response for set sync angle: {}.'.format(r))

    def set_sync_angle_ena_cgi(self, sync_angle=0):
        self.set_sync_angle_cgi(enable_flag=1, sync_angle=sync_angle)

    def set_sync_angle_dis_cgi(self, sync_angle=0):
        self.set_sync_angle_cgi(enable_flag=0, sync_angle=sync_angle)

    def get_sync_angle_cgi(self):
        # syn_value = '{"sync":' + str(enable_flag) + ',"syncAngle":' + str(sync_angle) + "}'"
        get_p = {
            "action": "get",
            "object": "lidar_sync"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        sync_value = r["Body"]["syncAngle"]
        if r["Body"]["sync"] in ['0', 0]:
            enable_state = 'disabled'
        elif r["Body"]["sync"] in ['1', 1]:
            enable_state = 'enabled'
        else:
            enable_state = 'unknown'
        # print(r["Body"])
        return enable_state, sync_value

    def get_code_range_cgi(self, info_type='code_range'):
        get_p = {
            "action": "get",
            "object": "lidar_data",
            "key": "code_range"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        if info_type == 'response_time':
            return response.elapsed.total_seconds()
        else:
            return r['Body']

    def set_code_range_cgi(self, code_mode, code1=127, code2_list_str='[132,148,164,180,140,156,172,188]'):
        code_mode_dict = {"random_code": '0', "fixed_code": '1', "fixed_code_manual": '2'}
        if code_mode_dict[code_mode] == '0':
            code_origin = self.get_code_range_cgi()
            code1 = str(code_origin['code1'])
            code2_list_str = '{}'.format(code_origin['code2_sp']).replace(' ', '')
        code_value = '{' + '"code1":{},'.format(code1) + '"code2":0,' \
                     + '"constant":{:s},'.format(code_mode_dict[code_mode]) \
                     + '"step":10,' + '"code2_sp":{:s}'.format(code2_list_str) + '}'
        # code_value = {'code1':127,'code2':0,'constant':0,'step':10,'code2_sp':[282,298,314,330,290,306,322,340]}
        set_p = {
            "action": "set",
            "object": "lidar_data",
            "key": "code_range",
            "value": code_value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        # print(r)

    def get_security_code_status_cgi(self):
        get_p = {
            "action": "get",
            "object": "lidar_data",
            "key": "security_code_status"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)

    def set_security_code_status_cgi(self):
        set_p = {
            "action": "set",
            "object": "lidar_data",
            "key": "security_code",
            "value": 921223
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        # print(r)

    def get_lidar_monitor_cgi(self, info_type="float"):
        # info_type = "float" or "str"
        get_p = {
            "action": "get",
            "object": "lidar_monitor"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if r['Head']['ErrorCode'] == '0':
            if info_type == 'response_time':
                return response.elapsed.total_seconds()
            elif info_type == "float":
                if "lidarInCur" in r["Body"]:
                    current = round(float(r["Body"]["lidarInCur"][:-3]) / 1000, 2)
                else:
                    current = np.nan
                if "lidarInVol" in r["Body"]:
                    voltage = round(float(r["Body"]["lidarInVol"][:-2]), 2)
                else:
                    voltage = np.nan
                if "lidarInPower" in r["Body"]:
                    power = round(float(r["Body"]["lidarInPower"][:-2]), 2)
                else:
                    power = np.nan
            elif info_type == "str":
                if "lidarInCur" in r["Body"]:
                    current = r["Body"]["lidarInCur"]
                else:
                    current = 'unknown'
                if "lidarInVol" in r["Body"]:
                    voltage = r["Body"]["lidarInVol"]
                else:
                    voltage = 'unknown'
                if "lidarInPower" in r["Body"]:
                    power = r["Body"]["lidarInPower"]
                else:
                    power = 'unknown'
            else:
                print("wrong info_type input")
                return None
            monitor_dict = {"current": current, "voltage": voltage, "power": power}
            return monitor_dict
        else:
            return None

    def set_fov_for_all_chn_cgi(self, min_value, max_value):
        if min_value > 3600 or min_value < 0 or max_value > 3600 or max_value < 0:
            print("wrong input please double check, value should be between 0 and 3600")
        fov_value = {"angle_setting_method": 0, "lidar_range": [min_value, max_value]}
        timeout = 10
        retry_num = 2
        files = None
        data = json.dumps(fov_value)
        s = requests.Session()
        a = requests.adapters.HTTPAdapter(max_retries=retry_num)
        s.mount('http://', a)
        post_url = self.base_url + "action=set&object=lidar_data&key=lidar_range"
        res = s.request('POST', post_url, data=data, files=files, timeout=timeout, cookies=self.cookies, verify=False)
        if res.status_code == 200:
            self.get_udp_num_psec()
        # assert res.status_code == 200, 'cannot connect to lidar, error code: %s' % (res.status_code)
        info = json.loads(res.text)
        info['Connection_Status'] = res.status_code
        return info

    def get_lidar_fov_cgi(self, info_type='fov'):
        if self.protocol_version in [4.3]:
            return None, None
        method_value_mapping = {0: 'for all channels', 1: 'for each channel (not for P128)',
                                2: 'multi-section FOV (not for P128)', 3: 'multi-section FOV (only for P128)'}
        get_p = {
            "action": "get",
            "object": "lidar_data",
            "key": "lidar_range"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if info_type == 'response_time':
            return response.elapsed.total_seconds()
        if r['Head']['ErrorCode'] == '0':
            if 'angle_setting_method' in r['Body']:
                method_value = r['Body']['angle_setting_method']
                # print("current FOV mode is {}".format(method_value_mapping[method_value]))
                return method_value, r['Body']
            else:
                return 0, r['Body']
        else:
            print(
                "cgi command not supported, getting FOV failed, please double check release/factory version and lidar")
            return None, None

    def set_clock_source_cgi(self, source):
        # todo: QT64, 128 has no clock selection options; PTP profile, time offset etc. excluded
        if source == 'GPS' or source == 'PTP':
            if source == 'GPS':
                source_value = 0
            else:
                source_value = 1
            set_p = {
                "action": "set",
                "object": "lidar",
                "key": "clock_source",
                "value": source_value
            }
            self.login_p128_release_cgi(r'admin', r'123456')
            if self.cs_flag and self.cookies:
                response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
            else:
                response = requests.get(self.base_url, set_p)
            r = json.loads(response.text)
            if source == 'GPS' and r['Head']['ErrorCode'] == '0':
                self.gen_gps_socket()
            elif r['Head']['ErrorCode'] != '0':
                print('clock source setting failed, please double check the lidar type!')
            # print('This is response for set clock_source: {}.'.format(r))

        else:
            print('wrong source value')

    def set_gps_data_format(self, data_format='GPRMC'):
        # data_format = 'GPRMC' / 'GPGGA'
        if data_format == 'GPRMC':
            data_format_value = 0
        elif data_format == 'GPGGA':
            data_format_value = 1
        else:
            print('wrong data_format input!')
            return
        set_p = {
            "action": "set",
            "object": "lidar",
            "key": "clock_data_format",
            "value": data_format_value}
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        # print('This is response for set gps_data_format: {}.'.format(r))

    def set_ptp_configuration_cgi(self, profile='1588v2'):
        ptp_profile = '{"Profile":0,"Domain":0,"Network":0,"LogAnnounceInterval":1,"LogSyncInterval":1,"LogMinDelayReqInterval":0,"tsn_switch":1}'
        gptp_profile = '{"Profile":1,"Domain":0,"Network":1,"LogAnnounceInterval":0,"LogSyncInterval":-3,"LogMinDelayReqInterval":0,"tsn_switch":1}'
        if profile == '1588v2':
            profile_value = ptp_profile
        elif profile == '802.1AS':
            profile_value = gptp_profile
        else:
            print('wrong ptp profile input!')
            return
        set_p = {
            "action": "set",
            "object": "lidar",
            "key": "ptp_configuration",
            "value": profile_value}
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        print('This is response for set ptp_configuration: {}.'.format(r))

    def set_clock_source_and_profile_cgi(self, source='PTP', profile='1588v2'):
        # source='PTP' profile='1588v2'/ '802.1AS'  source = 'GPS' profile='GPRMC' / 'GPGGA'

        if source == 'GPS' and profile in ['GPRMC', 'GPGGA']:
            self.set_clock_source_cgi(source=source)
            time.sleep(0.1)
            self.set_gps_data_format(data_format=profile)
        elif source == 'PTP' and profile in ['1588v2', '802.1AS']:
            self.set_clock_source_cgi(source=source)
            time.sleep(0.1)
            self.set_ptp_configuration_cgi(profile=profile)

    def set_code_p40_cgi(self, code1, code2):
        # todo: cgi request has no exception, only error code has difference
        payload = {
            "action": "set",
            "object": "lidar_data",
            "key": "security_code",
            "value": 921223
        }
        try:
            r = requests.get(self.base_url, payload)
        except Exception:
            return False
        valueWord = list('{"Low":115,"High":400,"L":[]}')
        for i in range(0, 40):
            channelWord = list('{"c":,"s":1,"i":}')
            if i % 2 == 0:
                channelWord.insert(5, str(code1))
            else:
                channelWord.insert(5, str(code2))
            channelWord.insert(-1, str(i))
            newWord = ''.join(channelWord)
            if i != 39:
                newWord = newWord + ','
            valueWord.insert(27 + i, newWord)
        finalWord = ''.join(valueWord)

        payload1 = {
            "action": "set",
            "object": "lidar_data",
            "key": "code_range",
            "value": finalWord
        }
        try:
            response = requests.get(self.base_url, payload1)
            r = json.loads(response.text)
        except Exception:
            return False
        payload2 = {
            "action": "get",
            "object": "lidar_data",
            "key": "code_range"
        }
        try:
            response = requests.get(self.base_url, payload2)
            r = json.loads(response.text)
            print('this is response for set P40 code:', r)
        except Exception:
            return False

        if r["Head"]["ErrorCode"] == '0' and r["Head"]["Message"] == 'Success':
            return True
        else:
            return False

    def set_code_p64_cgi(self, code1, code2):
        payload = {
            "action": "set",
            "object": "lidar_data",
            "key": "security_code",
            "value": 921223
        }
        try:
            r = requests.get(self.base_url, payload)
        except Exception:
            return False

        valueWord = list('{"Low":115,"High":400,"L":[]}')
        # channelWord = list('{"c":,"s":1,"i":}')
        for i in range(0, 64):
            channelWord = list('{"c":,"s":1,"i":}')
            if i % 2 == 0:
                channelWord.insert(5, str(code1))
            else:
                channelWord.insert(5, str(code2))
            channelWord.insert(-1, str(i))
            newWord = ''.join(channelWord)
            if i != 63:
                newWord = newWord + ','
            valueWord.insert(27 + i, newWord)
        finalWord = ''.join(valueWord)

        payload1 = {
            "action": "set",
            "object": "lidar_data",
            "key": "code_range",
            "value": finalWord
        }
        try:
            response = requests.get(self.base_url, payload1)
            r = json.loads(response.text)
        except Exception:
            return False
        payload2 = {
            "action": "get",
            "object": "lidar_data",
            "key": "code_range"
        }
        try:
            r = requests.get(self.base_url, payload2)
            r = json.loads(response.text)
            print('this is response for set P64 code:', r)

        except Exception:
            return False

        if r["Head"]["ErrorCode"] == '0' and r["Head"]["Message"] == 'Success':
            return True
        else:
            return False

    def get_code_p128_cgi(self):
        get_p = {
            "action": "get",
            "object": "lidar_data",
            "key": "code_range"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p)
        r = json.loads(response.text)
        # print(r)

    def set_rotate_direct_cgi(self, dirct):
        # 'clock': clockwise;  'anti-clock': anti-clockwise
        if dirct == 'clockwise':
            dirct_value = 0
        elif dirct == 'counterclockwise':
            dirct_value = 1
        else:
            print("wrong rotate direction input, set direction as clockwise")

        set_p = {
            "action": "set",
            "object": "lidar",
            "key": "rotate_direction",
            "value": dirct_value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        print('This is response for set rotate direction: {}.'.format(r))

        if r['Head']['ErrorCode'] == '0':
            if self.protocol_version in [3.1, 3.2, 3.4, 3.5, 3.22]:
                time.sleep(5)
            self.get_udp_num_psec()
            self.get_standard_resolution()
        else:
            print("regarding unknown error, setting rotation direction failed!")

    def set_rotate_dir_bool_cgi(self, dirct_value):
        # dirct_value = 0/1
        set_p = {
            "action": "set",
            "object": "lidar",
            "key": "rotate_direction",
            "value": dirct_value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        if r['Head']['ErrorCode'] == '0':
            print("rotation direction modified, response is: ", r)
            if self.protocol_version in [3.1, 3.2, 3.4, 3.5, 3.22]:
                time.sleep(5)
            self.get_udp_num_psec()
            self.get_standard_resolution()
        else:
            print("regarding unknown error, setting rotation direction failed!")

    def set_udp_seq_ena_cgi(self, ena_value=1):
        # only valid for P64 temporarily
        if self.protocol_version in [64.6] or self.laser_num == 40:
            set_p = {
                "action": "set",
                "object": "lidar",
                "key": "udp_sequence",
                "value": ena_value
            }
            self.login_p128_release_cgi(r'admin', r'123456')
            if self.cs_flag and self.cookies:
                response = requests.get(self.base_url, set_p, timeout=3, cookies=self.cookies, verify=False)
            else:
                response = requests.get(self.base_url, set_p, timeout=3)
            r = json.loads(response.text)
            print('This is response for set udp_sequence: {}.'.format(r))
        else:
            print("not supported in this type lidar!")

    def get_time_statistic_cgi(self, info_type='timestatistic'):
        if self.protocol_version in [4.1, 4.3]:
            # get_p = {
            #     "action": "get",
            #     "object": "timestatistic"
            # }
            get_p = {
                "action": "get",
                "object": "operationstatistics"
            }
        else:
            get_p = {
                "action": "get",
                "object": "TimeStatistic"
            }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if info_type == 'response_time':
            return response.elapsed.total_seconds()
        if r['Head']['ErrorCode'] == '0':
            work_temp = float(r['Body']['CurrentTemp'])
            startup_cnt = r['Body']['StartupTimes']
            totalworktime = r['Body']['TotalWorkingTime']
            tim_stat = [startup_cnt, work_temp, totalworktime]
            return tim_stat
        else:
            return [None, None, None]

    def get_dwn_reg_cgi(self, reg_add='43c088ac'):
        get_p = {
            "action": "get",
            "object": "register",
            "key": "down",
            "value": reg_add
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if r['Head']['ErrorCode'] == '0':
            reg_value = int(r['Body']['value'], 16)
            return reg_value
        else:
            print("register value not received! Error message is {}".format(r['Head']['Message']))
            return None

    def set_dwn_reg_cgi(self, reg_add="70000028", value=2):
        set_p = {
            "action": "set",
            "object": "down_register",
            "key": reg_add,
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        # print(r)
        if r['Head']['ErrorCode'] == '0':
            print("down-register {} set successfully".format(reg_add))
        elif "not support" in r["Head"]["Message"] or '3' in r["Head"]["ErrorCode"]:
            print("set register not support, please check whether fac version")
        else:
            print("unknown error, setting not successful")

    def set_up_reg_cgi(self, reg_add="70000028", value=2):
        # eye protection: 2: shutdown
        # hex_value = hex(value)
        set_p = {
            "action": "set",
            "object": "up_register",
            "key": reg_add,
            "value": value
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, set_p)
        r = json.loads(response.text)
        # print(r)
        if r['Head']['ErrorCode'] == '0':
            print("up-register {} set successfully".format(reg_add))
        elif "not support" in r["Head"]["Message"] or '3' in r["Head"]["ErrorCode"]:
            print("set register not support, please check whether fac version")
        else:
            print("unknown error, setting not successful")

    def get_up_reg_cgi(self, reg_add="1000f190"):
        get_p = {
            "action": "get",
            "object": "register",
            "key": "up",
            "value": reg_add
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if r['Head']['ErrorCode'] == '0':
            reg_value = int(r['Body']['value'], 16)
            return reg_value
        else:
            print("register value not received! Error message is {}".format(r['Head']['Message']))
            return None

    def get_spin_rate_cgi(self, info_type='spd'):
        get_p = {
            "action": "get",
            "object": "lidar_config"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if info_type == 'response_time':
            return response.elapsed.total_seconds()
        else:
            if r['Head']['ErrorCode'] == '0':
                spd_value = r['Body']['SpinSpeed']
                if spd_value == '2':
                    spin_spd = 600
                elif spd_value == '3':
                    spin_spd = 1200
                elif spd_value == '1':
                    spin_spd = 300
                elif spd_value == '4':
                    spin_spd = 200
                elif spd_value == '5':
                    spin_spd = 900
                elif spd_value == '6':
                    spin_spd = 750
                elif spd_value == '7':
                    spin_spd = 400
                elif spd_value == '8':
                    spin_spd = 500
                else:
                    spin_spd = -1
                    print("unknown spinning speed")
                return spin_spd
            else:
                return None

    def set_pwd_p128_web(self, WebCtrl):
        # WebCtrl is an instance of WebControl
        if self.cs_flag:
            cs_driver = WebCtrl.driver
            self.connectWeb_except_error_page(cs_driver, 60)
            # cs_driver.get(self.url)
            time.sleep(2)
            try:
                pwd_ele = cs_driver.find_element_by_id("password")
                pwd_ele.clear()
                pwd_ele.send_keys(str(123456))
                time.sleep(1)
                saveButton = cs_driver.find_element_by_id("submit")  ## not sure
                saveButton.click()
            except NoSuchElementException:
                pass
            time.sleep(1)
        if self.release:
            # requests.get(self.base_url[:-12], verify=False)
            try:
                cs_driver.get(self.url)
            except UnexpectedAlertPresentException:
                time.sleep(1)
                cs_driver.get(self.url)
        else:
            w_driver = WebCtrl.driver
            self.connectWeb_except_error_page(w_driver, 60)

    def get_statistic_cgi(self, info_type='dict', output_keys=None):
        get_p = {
            "action": "get",
            "object": "TimeStatistic"
        }
        if self.protocol_version in [4.1, 4.3]:
            # get_p = {
            #     "action": "get",
            #     "object": "timestatistic"
            # }
            get_p = {
                "action": "get",
                "object": "operationstatistics"
            }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r)
        if info_type == 'dict':
            temp_value = r['Body']['CurrentTemp']
            startup_counter = r['Body']['StartupTimes']
            totalworktime = r['Body']['TotalWorkingTime']
            statisic_dict = {'work_temp': temp_value, 'startup_counter': startup_counter,
                             'totalworktime': totalworktime}
            if 'SystemUptime' in r['Body']:
                SystemUptime = r['Body']['SystemUptime']
                statisic_dict.update({'SystemUptime': SystemUptime})
            return statisic_dict
        elif info_type == 'output_list' and output_keys:
            output_list = []
            for key_i in output_keys:
                if key_i in r['Body'].keys():
                    output_list.append(r['Body'][key_i])
                else:
                    output_list.append('n/a')
            return output_list

    def get_lidar_config_cgi(self, info_type='PTPStatus', output_keys=None):
        get_p = {
            "action": "get",
            "object": "lidar_config"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        # print(r['Body'])

        config_dict = r['Body']
        if info_type == 'response_time':
            return response.elapsed.total_seconds()
        elif info_type == 'PTPStatus':
            return config_dict['PTPStatus']
        elif info_type == 'PTPConfig':
            return config_dict['PTPConfig']
        elif info_type == 'GpsPort':
            return config_dict['GpsPort']
        elif info_type == 'NoiseFiltering':
            NoiseFiltering_state = ['off', 'on'][int(config_dict['NoiseFiltering'])]
            return NoiseFiltering_state
        elif info_type == 'ReflectivityMapping':
            ReflectivityMapping_state = ['off', 'on'][int(config_dict['ReflectivityMapping'])]
            return ReflectivityMapping_state
        elif info_type == 'gPTPConfig':
            return config_dict['gPTPConfig']
        elif info_type == 'PTPProfile':
            return config_dict['PTPProfile']
        elif info_type == 'ClockSource':
            source = ['GPS', 'PTP'][int(config_dict['ClockSource'])]
            return source
        elif info_type == 'DestIp':
            return config_dict['DestIp']
        elif info_type == 'all':
            return r['Body']
        elif info_type == 'output_list' and output_keys:
            output_list = []
            for key_i in output_keys:
                if key_i in r['Body'].keys():
                    output_list.append(r['Body'][key_i])
                else:
                    output_list.append('None')
            return output_list
        else:
            print("wrong input type input")
            return None

    def get_ptp_time_offset(self):
        ptp_status = self.get_lidar_config_cgi()
        if ':' in ptp_status:
            pstatus = ptp_status.split(':')[0].split('(')[0]
            return pstatus, ptp_status.split(':')[1].split(')')[0]
        else:
            return ptp_status, None

    def get_rotate_direct_cgi(self):
        get_p = {
            "action": "get",
            "object": "lidar_config"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        r = json.loads(response.text)
        rotdir_value = r['Body']['RotateDirection']
        if rotdir_value == '0':
            rot_dir = 'clockwise'
        elif rotdir_value == '1':
            rot_dir = 'counterclockwise'
        else:
            rot_dir = None
            print("wrong rotate direction, neither clockwise nor anti-clockwise")
        return rot_dir

    def set_code_qt(self, code=484):
        if not isinstance(code, int):
            print("wrong input type, please double check")
        elif code > 511 or code < 0:
            print("invalid code, please")
        else:
            set_p = {
                "action": "set",
                "key": code,
                "object": "anti_interference"
            }
            self.login_p128_release_cgi(r'admin', r'123456')
            if self.cs_flag and self.cookies:
                response = requests.get(self.base_url, set_p, cookies=self.cookies, verify=False)
            else:
                response = requests.get(self.base_url, set_p)
            r = json.loads(response.text)
            if r['Head']['ErrorCode'] == '0':
                get_code = self.get_code_qt()
                print("code setting successful, current code is {}".format(get_code))
            else:
                print("unknown error! response is {}".format(r))
                get_code = self.get_code_qt()
                print("code setting successful, current code is {}".format(get_code))

    def get_code_qt(self):
        get_p = {
            "action": "get",
            "object": "anti_interference",
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p)
        r = json.loads(response.text)
        if r['Head']['ErrorCode'] == '0':
            cur_code = r['Body']['anti_interference']
            print("code is {}".format(cur_code))
            return cur_code
        else:
            print("unknown Error, getting code failed")
            return None

    def post_upgrade_cgi(self, file_path):
        timeout = 30
        ufile = {'file': open(file_path, 'rb')}
        post_url = self.url + "/upgrade.cgi"
        if self.cs_flag:
            response = requests.post(post_url, files=ufile, timeout=timeout, cookies=self.cookies, verify=False)
        else:
            response = requests.post(post_url, files=ufile, timeout=timeout)
        res = json.loads(response.text)
        # print(res)
        if res['Head']['ErrorCode'] == '0':
            print("upgrade post requested successfully", datetime.now())
            return 1
        elif res['Head']['ErrorCode'] == '13':
            print("system recovering", datetime.now())
            time.sleep(3)
        else:
            print("response in post_upgrade_cgi", res)
            print("upgrade request rejected")
            # todo: log error and post current status
            return 0

    def get_status_cgi(self):
        get_p = {
            "action": "get",
            "object": "workmode"
        }
        # self.login_p128_release_cgi(r'admin', r'123456')
        try:
            self.login_p128_release_cgi(r'admin', r'123456')
        except AttributeError:
            # print("no login requested, cs_flag is {}".format(self.cs_flag))
            pass

        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        res = json.loads(response.text)
        # print(res['Body'])

        if self.protocol_version in [1.3, 1.4, 1.45, 6.1]:
            self.upgrade_phase = ['Sensor', 'Controller', 'Software', 'Parameter']
        elif self.protocol_version in [4.3]:
            self.upgrade_phase = ['Firmware', 'Software', 'Config_Parameter']
        else:
            self.upgrade_phase = ['Sensor', 'Controller', 'Software']

        if self.protocol_version in [1.3, 1.4, 1.45, 6.1]:
            p1, p2, p3, p4 = self.upgrade_phase
            key_list = ['{}_Process'.format(p1), '{}_Status'.format(p1), '{}_Process'.format(p2),
                        '{}_Status'.format(p2), '{}_Process'.format(p3), '{}_Status'.format(p3),
                        '{}_Process'.format(p4), '{}_Status'.format(p4)]
            workmode = res['Body']['WorkMode']
            if workmode == 1:
                upgrade_info = res['Body']['UpdateStatus']
                status_list = [dict_i['Status'] for dict_i in upgrade_info]
                value_list = [upgrade_info[0]['Process'], upgrade_info[0]['Status'],
                              upgrade_info[1]['Process'], upgrade_info[1]['Status'], upgrade_info[2]['Process'],
                              upgrade_info[2]['Status'], upgrade_info[3]['Process'], upgrade_info[3]['Status']]

                p1_upgrading_flag = (98 > int(upgrade_info[0]['Process']) > 1) and (int(upgrade_info[0]['Status']) < 2)
                p2_upgrading_flag = (98 > int(upgrade_info[1]['Process']) > 1) and (int(upgrade_info[1]['Status']) < 2)
                p3_upgrading_flag = (98 > int(upgrade_info[2]['Process']) > 1) and (int(upgrade_info[2]['Status']) < 2)
            else:
                return workmode, None, None, [0, 0, 0]
        else:
            p1, p2, p3 = self.upgrade_phase
            key_list = ['{}_Process'.format(p1), '{}_Status'.format(p1), '{}_Process'.format(p2),
                        '{}_Status'.format(p2), '{}_Process'.format(p3), '{}_Status'.format(p3)]
            workmode = res['Body']['WorkMode']
            if workmode == 1:
                upgrade_info = res['Body']['UpdateStatus']
                status_list = [dict_i['Status'] for dict_i in upgrade_info]
                value_list = [upgrade_info[0]['Process'], upgrade_info[0]['Status'], upgrade_info[1]['Process'],
                              upgrade_info[1]['Status'], upgrade_info[2]['Process'], upgrade_info[2]['Status']]

                p1_upgrading_flag = (98 > int(upgrade_info[0]['Process']) > 1) and (int(upgrade_info[0]['Status']) < 2)
                p2_upgrading_flag = (98 > int(upgrade_info[1]['Process']) > 1) and (int(upgrade_info[1]['Status']) < 2)
                p3_upgrading_flag = (98 > int(upgrade_info[2]['Process']) > 1) and (int(upgrade_info[2]['Status']) < 2)
            else:
                return workmode, None, None, [0, 0, 0]

        status_dict = dict(zip(key_list, value_list))
        # print(status_dict, status_list)
        return workmode, status_dict, status_list, [p1_upgrading_flag, p2_upgrading_flag, p3_upgrading_flag]

    def get_device_info_cgi(self, info_type='version', output_keys=None):
        # info_type: None, 'version', 'SN' etc.
        get_p = {
            "action": "get",
            "object": "device_info"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.base_url, get_p, timeout=1, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.base_url, get_p, timeout=1)
        res = json.loads(response.text)
        # print(res['Body'])
        # config_dict = res['Body']
        if res['Head']['ErrorCode'] != '0':
            print("cgi getting device_info failed, response: ", res)
            return []
        if info_type == 'response_time':
            return response.elapsed.total_seconds()
        elif info_type == 'version' and self.protocol_version in [4.1, 4.3]:
            sw_version = res['Body']['SW_Ver'].strip()
            fw_version = res['Body']['FW_Ver'].strip()
            UpFpga_parameter_version = res['Body']['UpFpgaParaVersion'].strip()
            UpFpga_para_config_version = res['Body']['UpFpgaParaConfigVersion'].strip()
            UpFpgaParaConfigSha = res['Body']['UpFpgaParaConfigSha'].strip()
            return [sw_version, fw_version, UpFpga_parameter_version, UpFpga_para_config_version, UpFpgaParaConfigSha]
        elif info_type == 'version':
            sw_version = res['Body']['SW_Ver'].strip()
            ctller_version = res['Body']['FW_Ver'].strip()
            sensor_version = res['Body']['Up_Fpga_Ver'].strip()
            return [sw_version, sensor_version, ctller_version]
        elif info_type == 'SN':
            return res['Body'][info_type]
        elif info_type == 'udp_sequence':
            return int(res['Body']['Udp_Seq'])
        elif info_type == 'output_list' and output_keys:
            output_list = []
            for key_i in output_keys:
                if key_i in res['Body'].keys():
                    output_list.append(res['Body'][key_i])
                else:
                    output_list.append('n/a')
            return output_list
        elif info_type == 'all':
            return res['Body']
        else:
            return res['Body']

    def upgrade_one_file(self, file):
        '''
        :param file:
        :return: cost_time of upgrade, if upgrade rejected, 0 will be returned
        '''
        file_name = os.path.basename(file)
        return_code = self.post_upgrade_cgi(file)
        start_time = time.time()
        cost_time = np.nan
        if return_code:
            for _ in range(800):
                time.sleep(1)
                try:
                    workmode, status_dict, status_list, phase_list = self.get_status_cgi()
                    # print(status_dict, status_list)
                    if sum(status_list) == 2 * len(status_list):
                        print("upgrade version: {} finished, {}".format(file_name, datetime.now()))
                        cost_time = time.time() - start_time
                        break
                except Exception as e:
                    print("unexpected error: ", e)
        else:
            print('upgrade rejected', datetime.now())
            return 0, np.nan, np.nan
        # ------------------reboot----------------
        udp_flag, extra_rbt_times, first_udp_time, fst_udp_seq = self.reboot_until_udp_received()
        print("debug: receive first udp time: ", datetime.now())
        for _ in range(5):
            try:
                self.__set_cs_flag__()
                break
            except:
                time.sleep(5)
        self.check_cgi_service_alive()
        print("debug: reach cgi first time at: ", datetime.now())
        self.check_cgi_service_alive()
        print("debug: reach cgi second time at: ", datetime.now())
        cgi_status = self.check_cgi_service_alive()
        print("debug: reach cgi third time at: ", datetime.now())

        if udp_flag and cgi_status:
            print("lidar wakes up and udp received")
            try:
                self.__get_binfo_from_udp__()
            except (json.decoder.JSONDecodeError, requests.exceptions.ConnectionError,
                    requests.exceptions.ConnectTimeout, requests.exceptions.ReadTimeout):
                print("CGI not reachable")
                time.sleep(10)
                self.__get_binfo_from_udp__()
            time.sleep(3)
            version = self.get_device_info_cgi(info_type='version')
            print("read version after upgrade from web is {}".format(version))
        else:
            try:
                version = self.get_device_info_cgi(info_type='version')
                print("read version after upgrade from web is {}".format(version))
            except:
                print("either no udp or cgi not reached after extra reboot")
        return cost_time, first_udp_time, fst_udp_seq

    def abnormal_one_file(self, file, poweroff_sec, total_time):
        '''
        :param file:
        :param poweroff_sec: predicted poweroff timing
        :param total_time:  self.upgrade_info_dict['total_time']
        :return:
        '''

        time_internal = 1  # in second
        tmp_status = self.post_upgrade_cgi(file)
        if tmp_status:
            t0 = time.time()
        else:
            return None, None, None, None, None, None, None

        poweroff_phase = None
        upgrading_flags = [0, 0, 0]
        udp_while_golden = None
        success_flag = 0
        mid_version = None

        while time.time() - t0 < poweroff_sec:
            time.sleep(time_internal - 0.02)
            workmode, status_dict, status_list, upgrading_flags = self.get_status_cgi()
            # print("status_dict: {}".format(status_dict))

        max_loop = round((total_time - poweroff_sec) / time_internal)
        for _ in range(max_loop):
            # if sensor upgrade process < 0: waiting
            if any(upgrading_flags):
                poweroff_phase = self.upgrade_phase[upgrading_flags.index(1)]
                print("poweroff_phase is {}, upgrading_flags: {}".format(poweroff_phase, upgrading_flags))
                # ----------------reboot-----------------
                udp_flag, extra_rbt_times, first_udp_time, fst_udp_seq = self.reboot_until_udp_received(extra_reboot=False)
                print("debug: receive first udp time: ", datetime.now())
                for _ in range(5):
                    try:
                        self.__set_cs_flag__()
                        break
                    except:
                        time.sleep(5)
                self.check_cgi_service_alive()
                print("debug: reach cgi first time at: ", datetime.now())
                self.check_cgi_service_alive()
                print("debug: reach cgi second time at: ", datetime.now())
                cgi_status = self.check_cgi_service_alive()
                print("debug: reach cgi third time at: ", datetime.now())
                # ---------------------------------------
                udp_while_golden = udp_flag
                if cgi_status and udp_flag:
                    self.__get_binfo_from_udp__()
                    mid_version = self.get_device_info_cgi(info_type='version')
                elif not cgi_status and udp_flag:
                    print("cgi not reachable in golden version")
                    # todo
                    # raise SystemExit("")
                else:
                    print("no udp sent nor cgi unreachable in golden version")
                break
            else:
                time.sleep(time_internal - 0.01)
                poweroff_sec = poweroff_sec + time_internal
                workmode, status_dict, status_list, upgrading_flags = self.get_status_cgi()
                print("extra timeloop: status_dict: {}".format(status_dict))

        re_upgrade_time, first_udp_time, fst_udp_seq = self.upgrade_one_file(file)
        cgi_status = self.check_cgi_service_alive()
        if not cgi_status:
            print('cgi not reached')

        return poweroff_sec, poweroff_phase, udp_while_golden, mid_version, re_upgrade_time, first_udp_time, fst_udp_seq

    def download_logs_cgi(self):
        # http: // 192.168.1.201 / download_log.cgi?action = get & object = app_logs
        self.download_url = self.url + '/download_log.cgi?'
        get_p = {
            "action": "get",
            "object": "app_logs"
        }
        self.login_p128_release_cgi(r'admin', r'123456')
        if self.cs_flag and self.cookies:
            response = requests.get(self.download_url, get_p, cookies=self.cookies, verify=False)
        else:
            response = requests.get(self.download_url, get_p)
        # res = json.loads(response.text)
        print(response.text)

    def upgrade_time_calculator(self, old_file, new_file):
        workmode, status_dict, status_list, phase_list = self.get_status_cgi()
        info_keys = ["{}_upgrade_needed".format(self.upgrade_phase[0]), "{}_time".format(self.upgrade_phase[0]),
                     "{}_upgrade_needed".format(self.upgrade_phase[1]), "{}_time".format(self.upgrade_phase[1]),
                     "{}_upgrade_needed".format(self.upgrade_phase[2]), "{}_time".format(self.upgrade_phase[2])]
        keys1 = ["{}_upgrade_needed".format(self.upgrade_phase[idx]) for idx in range(3)]
        keys2 = ['{}_Status'.format(self.upgrade_phase[idx]) for idx in range(3)]
        keys3 = ['{}_time'.format(self.upgrade_phase[idx]) for idx in range(3)]
        phase_keys = ['p1', 'p2', 'p3']

        info_dict = {}.fromkeys(info_keys, None)
        phase_dict = {}.fromkeys(phase_keys, None)

        old_file_name = os.path.basename(old_file)
        for _ in range(3):
            post_status = self.post_upgrade_cgi(old_file)
            if post_status:
                break
            else:
                print("upgrade version: {} failed, {}".format(old_file_name, datetime.now()))
                time.sleep(5)

        if post_status:
            for _ in range(500):
                time.sleep(1)
                workmode, status_dict, status_list, phase_list = self.get_status_cgi()
                if sum(status_list) == 2 * len(status_list):
                    print("upgrade version: {} finished, {}".format(old_file_name, datetime.now()))
                    break
            # ------------------reboot----------------
            udp_flag, extra_rbt_times, first_udp_time, fst_udp_seq = self.reboot_until_udp_received()
            if not udp_flag:
                return None, None
            print("lidar wakes up")
            time.sleep(5)
            # ----------------------------------------
        new_file_name = os.path.basename(new_file)
        self.__get_binfo_from_udp__()
        for _ in range(3):
            post_status = self.post_upgrade_cgi(new_file)
            if post_status:
                break
            else:
                print("upgrade version: {} failed, {}".format(new_file_name, datetime.now()))
                time.sleep(5)
        # post_status = self.post_upgrade_cgi(new_file)
        if not post_status:
            print('upgrade not really starts')
            return None, None

        t0 = time.time()
        status_dict = {}
        workmode = 0

        for _ in range(50):
            time.sleep(1)
            workmode, status_dict, status_list, phase_list = self.get_status_cgi()
            if workmode == 1:
                print('upgrade starts')
                # print(status_dict, status_list)
                break
            elif workmode == 0:
                print("upgrade not starts yet!")
                time.sleep(0.5)
            elif workmode == 2:
                print("Lidar is recovering itself")
                time.sleep(19)
            else:
                print('unknown status of work mode')
                time.sleep(1)

        if not status_dict:
            print("upgrade not started at least 75s after upgrade cgi command sent!")
            # todo: log error!
        elif workmode == 1:
            for idx in range(3):
                info_dict[keys1[idx]] = (status_dict[keys2[idx]] < 2) * 1
                phase_dict[phase_keys[idx]] = [(status_dict[keys2[idx]] < 2) * 1]
                # {'V1_target': {'p1': [1, 0, 54.7],
                #                'p2': [1, 54.7, 91.7],
                #                'p3': [1, 91.7, 100.4]},
                #  'target_V1': {'p1': [1, 0, 54.3],
                #                'p2': [1, 54.3, 91.2],
                #                'p3': [1, 91.2, 100.0]}}

            for _ in range(1000):
                time.sleep(0.5)
                workmode, status_dict, status_list, phase_list = self.get_status_cgi()

                if (not info_dict[keys3[0]]) and info_dict[keys1[0]] and status_dict[
                    '{}_Process'.format(self.upgrade_phase[0])] > 90 and status_dict[keys2[0]] == 2:
                    tmp = round(time.time() - t0, 1)
                    info_dict[keys3[0]] = tmp
                    phase_dict[phase_keys[0]] = phase_dict[phase_keys[0]] + [0, tmp]
                elif (info_dict[keys3[0]] is None) and (not info_dict[keys1[0]]):
                    info_dict[keys3[0]] = 0
                    phase_dict[phase_keys[0]] = phase_dict[phase_keys[0]] + [0, 0]

                if (not info_dict[keys3[1]]) and info_dict[keys1[1]] and status_dict[
                    '{}_Process'.format(self.upgrade_phase[1])] > 90 and status_dict[keys2[1]] == 2:
                    tmp = round(time.time() - t0, 1)
                    info_dict[keys3[1]] = tmp
                    phase_dict[phase_keys[1]] = phase_dict[phase_keys[1]] + [info_dict[keys3[0]], tmp]
                elif (info_dict[keys3[1]] is None) and (not info_dict[keys1[1]]):
                    info_dict[keys3[1]] = info_dict[keys3[0]]
                    phase_dict[phase_keys[1]] = phase_dict[phase_keys[1]] + [info_dict[keys3[0]], info_dict[keys3[0]]]

                if (not info_dict[keys3[2]]) and info_dict[keys1[2]] and status_dict[
                    '{}_Process'.format(self.upgrade_phase[1])] > 90 and status_dict[keys2[2]] == 2:
                    tmp = round(time.time() - t0, 1)
                    info_dict[keys3[2]] = tmp
                    phase_dict[phase_keys[2]] = phase_dict[phase_keys[2]] + [info_dict[keys3[1]], tmp]
                elif (info_dict[keys3[2]] is None) and (not info_dict[keys1[2]]):
                    info_dict[keys3[2]] = info_dict[keys3[1]]
                    phase_dict[phase_keys[2]] = phase_dict[phase_keys[2]] + [info_dict[keys3[1]], info_dict[keys3[1]]]

                if sum(status_list) == 2 * len(status_list):
                    print("upgrade version: {} finished, {}".format(old_file_name, datetime.now()))
                    break

            total_time = round((time.time() - t0), 1)
            info_dict.update({'total_time': total_time})
            print("debug: info_dict:", info_dict)
            # ------------------reboot----------------
            udp_flag, extra_rbt_times, first_udp_time, fst_udp_seq = self.reboot_until_udp_received()
            if not udp_flag:
                return None, None
            else:
                for _ in range(5):
                    try:
                        self.__set_cs_flag__()
                        break
                    except:
                        time.sleep(5)
                cgi_status = self.check_cgi_service_alive()
                if not cgi_status:
                    print('cgi not reached')
                    return None, None

            print("lidar wakes up")
            time.sleep(5)
            self.__get_binfo_from_udp__()
            return info_dict, phase_dict

        else:
            print('upgrade not really starts')
            return None, None

    def get_angle_calib(self):
        if self.protocol_version in [4.3]:
            r = self.ptc_sender(0x05, None)
            return r["response_payload"]
        else:
            r = self.ptc_sender(0x05, None)
            Angle_cal = r["response_payload"][0:].decode("utf-8")
            ac_list = [x.split(",") for x in Angle_cal.split("\n")]
            if len(ac_list) > self.laser_num + 1:
                angle_cal = pd.DataFrame(ac_list[1:self.laser_num + 1], columns=ac_list[0])
            else:
                angle_cal = pd.DataFrame(ac_list[1:], columns=ac_list[0])
            #  if len(angle_cal)>self.lasernum:

            angle_cal.columns = ["Channel", "Elevation", "Azimuth"]
            return angle_cal

    def abandon_first_udp_data(self):
        # self.get_udp_num_psec()
        for _ in range(15000):
            data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)

    def abandon_udp_datas_in_cache(self):
        # time_list = []
        for idx in range(1000):
            t0 = time.time()
            for _ in range(103):
                data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
            t1 = time.time()
            # time_list.append((t1 - t0) * 1e4)
            if (t1 - t0) * 1e4 > self.delta_timestamp_udp:
                # print(time_list)
                print("there are about {} udp datas in cache".format(103 * idx))
                return
        print("there are over 101000 udp datas in cache, please check your cache size")
        return

    def abandon_udp_datas_in_cache_offline(self):
        '''
        with no lidar connected, all cloud point udps in cache sent by this lidar will be cleaned; this func
        blocks for max. 3 sec
        :return:
        '''
        t0 = time.time()
        idx = 0
        self.udpsock.settimeout(1)
        while time.time() - t0 < 3:
            try:
                data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
                idx += 1
            except socket.timeout:
                self.udpsock.settimeout(180)
                print("there are about {} udp datas in cache".format(idx))
                return
        print("there are over 100000 udp datas in cache, please check your lidar connection and cache size")
        return

    def first_udp_time_rbt(self, t0, extra_rbt_flag=1, debug_mode=False):
        seq_idx = [self.tail_index_dict['udp_sequence'][0],
                   self.tail_index_dict['udp_sequence'][0] + self.tail_index_dict['udp_sequence'][1]]
        # print(seq_idx)
        seq_limit = 5 + 12000 * (self.protocol_version in [6.1]) + 15000 * (self.protocol_version in [4.1]) + \
                    6000 * (self.protocol_version in [4.3])
        udp_seq = -1
        self.udpsock.settimeout(120)
        while time.time() - t0 < 120:
            try:
                data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
                if len(data) == self.data_size:
                    t_i = time.time()
                    udp_seq = struct.unpack('<I', data[seq_idx[0]:seq_idx[1]])[0]
                    if udp_seq < seq_limit:
                        # for _ in range(seq_limit+5):
                        #     data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
                        self.udpsock.settimeout(180)
                        print('received first udp at {}s, first sequence is {}'.format(t_i - t0, udp_seq))
                        return t_i - t0, data, udp_seq
                else:
                    udp_seq = -1
            except socket.timeout:
                print("no cloudpoints udp received in 120 seconds after reboot, this reboot failed!")
                if debug_mode:
                    self.udpsock.settimeout(180)
                    return 120, None, -1
                elif extra_rbt_flag:
                    self.relay.set_relay(status='off')
                    time.sleep(10)
                    self.relay.set_relay(status='on')
                    time.sleep(self.reboot_time)
                self.udpsock.settimeout(180)
                return 120, None, -1

        print("first udp sequence exceeds limit ", datetime.now())
        self.udpsock.settimeout(180)
        return 120.1, None, udp_seq

    def check_cgi_service_alive(self):
        for _ in range(20):
            try:
                spin_speed = self.get_spin_rate_cgi()
                return_mode_value, return_mode, dual_flag = self.get_return_mode_cgi()
                version = self.get_device_info_cgi(info_type='version')
                if self.protocol_version not in [4.3]:
                    method_value, r = self.get_lidar_fov_cgi()
                    if all([spin_speed, return_mode_value, return_mode, version, r]):
                        return 1
                else:
                    if all([spin_speed, return_mode_value, return_mode, version]):
                        return 1
            except (json.decoder.JSONDecodeError, requests.exceptions.ConnectTimeout,
                    requests.exceptions.ConnectionError):
                time.sleep(2)
                print('connection_timeout')
            except requests.exceptions.ReadTimeout:
                time.sleep(1.5)
                print('ReadTimeout')
            except TypeError:
                time.sleep(1.5)
                print("different_format")
            except Exception as e:
                time.sleep(2)
                print("unexpected error in check_cgi_service_alive: ", e)
        return 0

    def reboot_until_udp_received(self, rbt_type='hard', extra_reboot=True):
        extra_rbt_times = 0
        fst_udp_seq = None
        print("power off")
        if rbt_type == 'hard':
            self.relay.set_relay(status='off')
            time.sleep(10)
            self.relay.set_relay(status='on')
        elif rbt_type == 'soft':
            self.set_reboot_cgi()
        else:
            print('wrong rbt_type input!')
            return None, None, None

        tmp = time.time()
        first_udp_time, data, fst_udp_seq = self.first_udp_time_rbt(tmp, extra_rbt_flag=0)
        if first_udp_time < 120:
            udp_flag = 1
            for _ in range(5):
                data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
            return udp_flag, extra_rbt_times, first_udp_time, fst_udp_seq
        else:
            udp_flag = 0
            extra_rbt_times = 0

        if not extra_reboot:
            return udp_flag, extra_rbt_times, first_udp_time, fst_udp_seq

        while first_udp_time > 119 and extra_rbt_times < 10:
            if rbt_type == 'hard':
                self.relay.set_relay(status='off')
                time.sleep(10)
                self.relay.set_relay(status='on')
            elif rbt_type == 'soft':
                self.set_reboot_cgi()
            else:
                print('wrong rbt_type input!')
                return None, None, None

            tmp = time.time()
            first_udp_time, data, fst_udp_seq = self.first_udp_time_rbt(tmp, extra_rbt_flag=0)
            if data:
                print('extra reboot successfully!')
                udp_flag = 1
                return udp_flag, extra_rbt_times, first_udp_time, fst_udp_seq
            else:
                print('extra reboot also failed!!')
                extra_rbt_times += 1
                # todo: log warning: extra reboot once

        return udp_flag, extra_rbt_times, first_udp_time, fst_udp_seq

    def channels_in_distance_range_check(self, distance_range, invert=False):
        '''
        figure out all channels with measured distances within/out the defined distance_range.
        invert == True: channels never have the defined distance_range will be picked out. measurement time window: 1s
        找出所有通道包含（任一）/从未包含（从未出现）输入的距离范围的距离
        例如 瞎线检查： Alydar.distance_channels_check(distance_range=[0.1, 250], invert=True)
        :param distance_range: in list， ex: [3, 4]
        :param invert:
        :return: channels in list
        '''
        # for figuring out blind channels with distance_range=[0.1, 250], invert=True
        block1_start_idx, block_bytes, block_interval, channel_bytes = self.block_info
        distance_matrix = np.zeros((self.udp_num_psec_rm * self.block_num, self.laser_num))
        if channel_bytes == 3:
            a_dtype = np.dtype([('distance', '<u2'), ('intensity', 'u1')])
        elif channel_bytes == 4:
            a_dtype = np.dtype([('distance', '<u2'), ('intensity', 'u1'), ('confidence', 'u1')])
        else:
            print('unknown error')
            return None

        all_chn = list(range(self.laser_num))
        datas = []
        idx = 0
        while idx < self.udp_num_psec_rm:
            data, addr = self.udpsock.recvfrom(self.data_size + 42)
            if len(data) == self.data_size:
                datas.append(data)
                idx += 1

        for data_idx, data in enumerate(datas):
            for idx in range(self.block_num):
                tmp_data = data[
                           block1_start_idx + block_interval * idx:block1_start_idx + block_bytes + block_interval * idx]
                tmp_array = np.frombuffer(tmp_data, dtype=a_dtype)
                distance_matrix[data_idx * self.block_num + idx, :] = tmp_array['distance'] * self.dist_unit

        dist_low, dist_high = distance_range

        # dist_sum = distance_matrix.sum(axis=0)  # sum of column
        # col_idx = list(set(list(np.where(dist_sum < 0.1)[0])))

        row_idx, chn_idx_i = np.where(np.logical_and(distance_matrix > dist_low, distance_matrix < dist_high))
        chn_idx = list(set(list(chn_idx_i)))

        if invert:
            invert_chn = [chn for chn in all_chn if chn not in chn_idx]
            return invert_chn
        else:
            return chn_idx

    def gen_array_from_datas(self, datas, id_range):
        id_min, id_max = id_range
        row_num = round(len(datas) * self.block_num)

        block1_start_idx, block_bytes, block_interval, channel_bytes = self.block_info
        laser_num = id_max - id_min + 1
        distance_matrix = np.zeros((row_num + 1, laser_num + 1))
        intensity_matrix = np.zeros((row_num + 1, laser_num + 1))
        if channel_bytes == 3:
            a_dtype = np.dtype([('distance', '<u2'), ('intensity', 'u1')])
        elif channel_bytes == 4:
            a_dtype = np.dtype([('distance', '<u2'), ('intensity', 'u1'), ('confidence', 'u1')])
        else:
            print('unknown error')
            return None

        distance_matrix[0, 1:] = np.arange(id_min, id_max + 1)
        intensity_matrix[0, 1:] = np.arange(id_min, id_max + 1)
        azms = []
        for data_idx, data in enumerate(datas):
            for idx in range(self.block_num):
                tmp_data = data[block1_start_idx + block_interval * idx:
                                block1_start_idx + block_bytes + block_interval * idx]
                tmp_array = np.frombuffer(tmp_data, dtype=a_dtype)
                distance_matrix[data_idx * self.block_num + idx + 1, 1:] = \
                    tmp_array['distance'][id_min:id_max + 1] * self.dist_unit
                intensity_matrix[data_idx * self.block_num + idx + 1, 1:] = tmp_array['intensity'][id_min:id_max + 1]

            for idx1, idx2 in self.azmth_idx_dict.values():
                azms.append(data[idx1:idx1 + 2])

        stctstr = '<' + 'H' * row_num
        azm_tmp = struct.unpack(stctstr, b''.join(azms))
        distance_matrix[1:, 0] = np.array(azm_tmp)
        intensity_matrix[1:, 0] = np.array(azm_tmp)
        # print("debug: ", len(azm_tmp), azm_tmp[:8])

        return distance_matrix, intensity_matrix

    def gen_dist_int_matrix_online(self, fov_range, id_range, udp_num=None, write_flag=False):
        '''
        if this function applied for AT, the mapping between rotor azimuth and emission light azimuth should be
        considered
        :param fov_range: [30, 60]
        :param id_range: laser ID range, 2-127, [33, 90]
        :return:
        '''
        fov_min, fov_max = fov_range
        # id_min, id_max = id_range
        if not udp_num:
            return_num = self.get_return_num()
            udp_num = np.ceil((fov_max - fov_min) * 100 / self.resolution_per_udp * (return_num / 2))

        # -----------------figure out first udp position----------------
        idx1, idx2 = self.azmth_idx_dict['azmth1']
        for _ in range(self.udp_num_psec_rm):
            data, addr = self.udpsock.recvfrom(int(self.data_size) + 42)
            if len(data) == self.data_size:
                azm_i = struct.unpack('<H', data[idx1:idx1 + 2])[0]
                if fov_min * 100 - (self.resolution_per_udp * 1.6) <= azm_i <= fov_min * 100 - (
                        self.resolution_per_udp * 0.4):
                    # print("debug: ", azm_i)
                    break

        # -----------------receive udps----------------
        datas = self.receive_multi_udps(udp_num)

        # -----------------build array---------------
        distance_matrix, intensity_matrix = self.gen_array_from_datas(datas, id_range)

        # write into csv
        if write_flag:
            record_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            dist_csv = '/home/hesai/WORK/function_debug/temp/distance_matrix_{}.csv'.format(record_time)
            int_csv = '/home/hesai/WORK/function_debug/temp/intensity_matrix_{}.csv'.format(record_time)
            # np_header = ','.join([str(id) for id in range(id_min, id_max+1)])
            np.savetxt(dist_csv, distance_matrix)  # , comments='')  #  , fmt=fmt_i) , header=All_in_One.np_header)
            np.savetxt(int_csv, intensity_matrix)

        return distance_matrix[1:, 1:], intensity_matrix[1:, 1:]

    def noise_check(self, distance_array, dist_max=[40, 50], special_dist=[15, 50]):
        tolerance = 0.5
        max_dist, max_count_limit = dist_max
        spec_dist, spec_count_limit = special_dist

        noise_count = np.sum(distance_array > max_dist)
        noise_rate = noise_count / distance_array.size
        noise_flag = (noise_count > max_count_limit)

        spec_count = np.sum(
            np.logical_and(distance_array > (spec_dist - tolerance), distance_array < (spec_dist + tolerance)))
        spec_rate = spec_count / distance_array.size
        spec_flag = (spec_count > spec_count_limit)

        print(noise_count, noise_flag, noise_rate, spec_count, spec_flag, spec_rate)
        return noise_count, noise_flag, spec_count, spec_flag

    def dist_check(self, distance_array, spec_info, noise_info):
        # spec_info = [14.5, 15.5, 50], noise_info = [40, 250, 50]
        # input = {'range_1': [14.0, 16.0], 'limit_1': 50, 'range_2': [50.0, 500.0], 'limit_2': 100}
        spec_dist_min, spec_dist_max, spec_max_count = spec_info
        noise_dist_min, noise_dist_max, noise_max_count = noise_info

        spec_count = np.sum(np.logical_and(distance_array > spec_dist_min, distance_array < spec_dist_max))
        spec_flag = (spec_count > spec_max_count)

        noise_count = np.sum(np.logical_and(distance_array > noise_dist_min, distance_array < noise_dist_max))
        noise_flag = (noise_count > noise_max_count)

        # spec_result = [spec_count, spec_flag]
        # noise_result = [noise_count, noise_flag]
        # print("debug: special distance:", spec_count, spec_flag, " noise:", noise_count, noise_flag)
        return spec_count, spec_flag, noise_count, noise_flag

    def dist_check_pretest(self, distance_array, spec_range, noise_range):
        spec_dist_min, spec_dist_max = spec_range
        noise_dist_min, noise_dist_max = noise_range

        spec_count = np.sum(np.logical_and(distance_array > spec_dist_min, distance_array < spec_dist_max))
        noise_count = np.sum(np.logical_and(distance_array > noise_dist_min, distance_array < noise_dist_max))

        return spec_count, noise_count

    def gen_dist_int_matrix_offline(self, pcap_file, fov_range, id_range, start_pos=0, udp_num=None):
        fov_min, fov_max = fov_range
        if not udp_num:
            udp_num = np.ceil((fov_max - fov_min) * 100 / self.resolution_per_udp * (self.return_num / 2))

        idx1, idx2 = self.azmth_idx_dict['azmth1']

        if start_pos:
            first_upos = start_pos
        else:
            data, first_upos = self.pick_out_first_udp(pcap_file)

        idx = 0
        with open(pcap_file, 'rb') as file_handle:
            for data_type, data in self.read_pcap_ut_in_pieces_generator(file_handle, start_pos=first_upos):
                idx += 1
                if len(data) == self.data_size:
                    azm_i = struct.unpack('<H', data[idx1:idx1 + 2])[0]
                    # print("debug: ", azm_i)
                    if fov_min * 100 - (self.resolution_per_udp * 1.6) <= azm_i <= fov_min * 100 - (
                            self.resolution_per_udp * 0.4):
                        # print("debug: ", azm_i)
                        cur_pos = file_handle.tell()
                        break
                if idx > self.udp_num_psec_rm:
                    cur_pos = file_handle.tell()
                    break

        # -----------------receive udps----------------
        datas, cur_pos = self.collect_udps_from_pcap(pcap_file, cur_pos, udp_num)

        # -----------------build array---------------
        distance_matrix, intensity_matrix = self.gen_array_from_datas(datas, id_range)

        return distance_matrix, intensity_matrix, cur_pos

    def array_statistic_analyse(self, array_i):
        # array_size = array_i.size
        a_array = array_i[1:, 1:]
        subarray = a_array[np.nonzero(a_array)]
        nonzero_count = np.count_nonzero(a_array)
        para_mean = np.mean(subarray)
        para_std = np.std(subarray)

        result_arr = np.array([nonzero_count, para_mean, para_std])

        chn = array_i[0][1:].astype('int64')
        tmp = chn * (np.sum(a_array, axis=0) == 0)
        blind_chn = list(tmp[np.flatnonzero(tmp)])

        return result_arr, blind_chn

    def gen_pc_performance_pretest(self, pcap_file, fov_range, id_range, spec_range, noise_range):
        data, rpos = self.pick_out_first_udp(pcap_file)

        result_list = []
        for _ in range(10):
            if self.protocol_version in [4.3]:
                fov_range = [38, 330]
                distance_matrix, intensity_matrix, cur_pos = \
                    self.gen_dist_int_matrix_offline(pcap_file, fov_range, id_range, start_pos=rpos, udp_num=3600)
            else:
                distance_matrix, intensity_matrix, cur_pos = \
                    self.gen_dist_int_matrix_offline(pcap_file, fov_range, id_range, start_pos=rpos)

            rpos = cur_pos
            scene_info, blind_chns = self.array_statistic_analyse(distance_matrix)
            nonzero_count, para_mean, para_std = scene_info
            spec_count, noise_count = \
                self.dist_check_pretest(distance_matrix[1:, 1:], spec_range=spec_range, noise_range=noise_range)
            result_list.append([nonzero_count, para_mean, para_std, blind_chns, spec_count, noise_count])

        return result_list

    def abstract_scene_character_to_Lydar(self, result_list):
        dist_list = []
        blind_chns_list = []
        for nonzero_count, para_mean, para_std, blind_chns, spec_count, noise_count in result_list:
            dist_list.append([nonzero_count, para_mean, para_std, spec_count, noise_count])
            blind_chns_list.extend(blind_chns)
        self.blind_chns = set(blind_chns_list)

        tmp = np.mean(np.array(dist_list), axis=0)
        self.nonzero_count, self.para_mean, self.para_std, self.spec_count, self.noise_count = tmp
        self.standard_dist_arr = np.array([self.nonzero_count, self.para_mean, self.para_std])
        print("debug: standard scene", self.standard_dist_arr)

    def pc_performance_check(self, pcap_file, fov_range, id_range, spec_range, spec_limit, noise_range, noise_limit):
        dist_list = []
        blind_chns_list = []
        spec_count_list = []
        spec_flag_list = []
        noise_count_list = []
        noise_flag_list = []

        data, rpos = self.pick_out_first_udp(pcap_file)
        spec_info = spec_range + [spec_limit]
        noise_info = noise_range + [noise_limit]
        loops = 10 - 3 * (self.protocol_version in [4.3])
        for _ in range(loops):
            if self.protocol_version in [4.3]:
                # fov_range = [38, 330]
                distance_matrix, intensity_matrix, cur_pos = \
                    self.gen_dist_int_matrix_offline(pcap_file, fov_range, id_range, start_pos=rpos, udp_num=3600)
            else:
                distance_matrix, intensity_matrix, cur_pos = \
                    self.gen_dist_int_matrix_offline(pcap_file, fov_range, id_range, start_pos=rpos)

            rpos = cur_pos
            scene_info, blind_chns = self.array_statistic_analyse(distance_matrix)
            print("debug: ", scene_info, blind_chns)
            spec_count_i, spec_flag_i, noise_count_i, noise_flag_i = \
                self.dist_check(distance_matrix[1:, 1:], spec_info=spec_info, noise_info=noise_info)
            dist_list.append(list(scene_info))
            blind_chns_list.extend(blind_chns)
            spec_count_list.append(spec_count_i)
            spec_flag_list.append(spec_flag_i)
            noise_count_list.append(noise_count_i)
            noise_flag_list.append(noise_flag_i)

        blind_channels = list(set(blind_chns_list) - self.blind_chns)
        tmp = np.array(dist_list) / self.standard_dist_arr
        dist_ratio = np.mean(tmp, axis=0)
        dist_fail_flag = np.any(np.logical_or(tmp > 1.08, tmp < 0.92))  # True: failed
        spec_count = np.mean(spec_count_list)
        spec_fail_flag = any(spec_flag_list)  # True: special distance range exceeds limit
        noise_count = np.mean(noise_count_list)
        noise_fail_flag = any(noise_flag_list)  # True: noise exceeds limit

        return [blind_channels, dist_ratio - 1, dist_fail_flag, spec_count, spec_fail_flag, noise_count,
                noise_fail_flag]


def gen_timestamp(time_bytes):
    if len(time_bytes) == 6:
        """
        解析按 年，月，日，时，分，秒 排列的UTC时间戳到timestamp
        """
        detail_time = struct.unpack("<BBBBBB", time_bytes)
        y = detail_time[0]
        if y >= 100:
            year = y + 1900
        else:
            year = y + 1970
        month = detail_time[1]
        day = detail_time[2]
        hour = detail_time[3]
        minute = detail_time[4]
        second = detail_time[5]
        if month == 0 or day == 0:
            ts = 0
        else:
            dt = datetime(year, month, day, hour, minute, second)
            dt = dt.replace(tzinfo=timezone.utc)
            ts = dt.timestamp()
        return int(ts)
    elif len(time_bytes) == 4:
        """
        解析包中timestamp，时间为微秒
        """
        detail_time = struct.unpack("<I", time_bytes)
        ts = detail_time[0]
        return int(ts)
    else:
        return 0


if __name__ == "__main__":
    relay_host = "192.168.1.210"
    relay_port = 8089
    relay_channel = "1"
    host = '192.168.1.201'
    port = 2368

    from TestFuntion.GeneralFunction import RelayController

    pcap_file = '/home/hesai/Downloads/rbt_loop_3_2021_12_27_10_45_35.pcap'

    UDV = Lydar(host, port)
    UDV.__get_binfo_from_udp__()

    UDV.relay = RelayController(relay_host, relay_port, relay_channel)
    print('UDV binds with relay')

    # response = UDV.ptc_sender(0x7F, b'\xee\xff\x01')
    # print(response)
    print(UDV.sha)
    print(UDV.para_sha)

    print(UDV.delta_timestamp_udp)

    UDV.abandon_udp_datas_in_cache()
    UDV.abandon_udp_datas_in_cache_offline()

    UDV.relay.set_relay(status='off')
    time.sleep(0.5)
    UDV.abandon_udp_datas_in_cache_offline()

    UDV.relay.set_relay(status='on')

    print(111)
    time.sleep(500)

    print(UDV.azmth_idx_dict)
    tmps = []
    loop = 30000
    print(UDV.udp_num_psec_rm)
    for _ in range(loop):
        data, addr = UDV.udpsock.recvfrom(int(UDV.data_size) + 42)
        if len(data) == UDV.data_size:
            for idx1, idx2 in UDV.azmth_idx_dict.values():
                tmps.append(data[idx1:idx1 + 2])
    stctstr = '<' + 'H' * len(tmps)
    azm_list = list(set(struct.unpack(stctstr, b''.join(tmps))))
    azm_list.sort()
    azm_set = [-30, *azm_list, 36030]
    end_angle = [azm_set[i] for i in range(len(azm_set) - 1) if azm_set[i + 1] - azm_set[i] > 20]
    start_angle = [azm_set[i + 1] for i in range(len(azm_set) - 1) if azm_set[i + 1] - azm_set[i] > 20]
    firing_area = [start_angle[:-1], end_angle[1:]]
    print(firing_area)

    idx1, idx2 = UDV.azmth_idx_dict['azmth1']
    for _ in range(loop):
        data, addr = UDV.udpsock.recvfrom(int(UDV.data_size) + 42)
        if len(data) == UDV.data_size:
            azm_i = struct.unpack('<H', data[idx1:idx1 + 2])[0]
            if azm_i >= 3990 and azm_i <= 4000:
                print(azm_i)
                break

    tmp = []
    for _ in range(10):
        data, addr = UDV.udpsock.recvfrom(int(UDV.data_size) + 42)
        tmp.append(data[idx1:idx1 + 2])

    stctstr = '<' + 'H' * 10
    azm_list = struct.unpack(stctstr, b''.join(tmp))
    print(azm_list)

    time.sleep(5000)

    # UDV.__instance_via_offline__(pcap_file)
    # print('UDV setup successful')

    idx = 0
    with open(pcap_file, 'rb') as file_handle:
        for data_type, data in UDV.read_pcap_ut_in_pieces_generator(file_handle):
            idx += 1
            if data_type == 'u' and len(data) == UDV.data_size:
                print(UDV.parse_udp_all_qt34_35(data), idx)
                time.sleep(0.1)
            # print(UDV.parse_udp_tail_general(data, data_type='all', return_type="dict"))
            else:
                time.sleep(0.2)
                # print(data_type, data, idx)

    time.sleep(5000)

    # raise SystemExit('reboot failed!')

    print(111)
    time.sleep(500)

    #
    # print(UDV.host, UDV.port)
    # time.sleep(10)
    # UDV.set_lidar_ip_udp_mask_cgi(host)
    # UDV.set_destination_ip_udp_port_gps_cgi(port)
    # print(UDV.host, UDV.port)
