'''
cd /dev
ls | grep USB
一般会显示: ttyUSB0
sudo chmod 666 /dev/ttyUSB0
'''



import serial
import serial.tools.list_ports
from struct import unpack, pack

class Communication():

    #初始化
    def __init__(self,com,bps,timeout,bytesize = 8,parity = 0,stopbits = 1):
        self.port = com
        self.bps = bps
        self.timeout =timeout
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        global Ret
        try:
            # 打开串口，并得到串口对象
             self.main_engine= serial.Serial(self.port,self.bps,timeout=self.timeout)
            # 判断是否打开成功
             if (self.main_engine.is_open):
               Ret = True
        except Exception as e:
            print("---异常---：", e)

    # 打印设备基本信息
    def Print_Name(self):
        print(self.main_engine.name) #设备名字
        print(self.main_engine.port)#读或者写端口
        print(self.main_engine.baudrate)#波特率
        print(self.main_engine.bytesize)#字节大小
        print(self.main_engine.parity)#校验位
        print(self.main_engine.stopbits)#停止位
        print(self.main_engine.timeout)#读超时设置
        print(self.main_engine.writeTimeout)#写超时
        print(self.main_engine.xonxoff)#软件流控
        print(self.main_engine.rtscts)#软件流控
        print(self.main_engine.dsrdtr)#硬件流控
        print(self.main_engine.interCharTimeout)#字符间隔超时

    #打开串口
    def Open_Engine(self):
        self.main_engine.open()

    #关闭串口
    def Close_Engine(self):
        self.main_engine.close()
        print(self.main_engine.is_open)  # 检验串口是否打开

    # 打印可用串口列表
    @staticmethod
    def Print_Used_Com():
        port_list = list(serial.tools.list_ports.comports())
        print(port_list)





    #接收指定大小的数据
    #从串口读size个字节。如果指定超时，则可能在超时后返回较少的字节；如果没有指定超时，则会一直等到收完指定的字节数。
    def Read_Size(self,size):
        return self.main_engine.read(size=size)

    #接收一行数据
    # 使用readline()时应该注意：打开串口时应该指定超时，否则如果串口没有收到新行，则会一直等待。
    # 如果没有超时，readline会报异常。
    def Read_Line(self):
        return self.main_engine.readline()

    #发数据
    def Send_data(self,data):
        self.main_engine.write(data)

    #更多示例
    # self.main_engine.write(chr(0x06).encode("utf-8"))  # 十六制发送一个数据
    # print(self.main_engine.read().hex())  #  # 十六进制的读取读一个字节
    # print(self.main_engine.read())#读一个字节
    # print(self.main_engine.read(10).decode("gbk"))#读十个字节
    # print(self.main_engine.readline().decode("gbk"))#读一行
    # print(self.main_engine.readlines())#读取多行，返回列表，必须匹配超时（timeout)使用
    # print(self.main_engine.in_waiting)#获取输入缓冲区的剩余字节数
    # print(self.main_engine.out_waiting)#获取输出缓冲区的字节数
    # print(self.main_engine.readall())#读取全部字符。

    #接收数据
    #一个整型数据占两个字节
    #一个字符占一个字节

    def Recive_data(self,way=1):   # 读取串口数据
        # 循环接收数据，此为死循环，可用线程实现
        # print("开始接收数据：")
        while True:
            try:
                # 一个字节一个字节的接收
                if self.main_engine.in_waiting:
                    if(way == 0):
                        for i in range(self.main_engine.in_waiting):
                            print("接收ascii数据："+str(self.Read_Size(1)))
                            data1 = self.Read_Size(1).hex()#转为十六进制
                            data2 = int(data1,16)#转为十进制print("收到数据十六进制："+data1+"  收到数据十进制："+str(data2))
                    if(way == 1):
                        #整体接收
                        # data = self.main_engine.read(self.main_engine.in_waiting).decode("utf-8")#方式一
                        data = self.main_engine.read_all()#方式二
                        # print("接收ascii数据：", data)
                        return data
            except Exception as e:
                print("异常报错：",e)
     #crc16校验码生成
    def calc_crc(self,string):    # crc校验
        data = bytearray.fromhex(string)
        crc = 0xFFFF  # 初始值
        for pos in data:
            crc ^= pos  # 每次两位16进制和crc低8位异或
            for i in range(8):  # 右移八次
                if ((crc & 1) != 0):  # 如果移出去的不是0（即移出去的是1）则右移1位且和A001异或。否则为0直接右移1位不作其他处理
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        crc_code = hex(((crc & 0xff) << 8) + (crc >> 8))[2:]
        if len(crc_code) < 4:
            crc_code = (4-len(crc_code)) * '0' + crc_code
        return crc_code   # 高低8位对调。

    def check_temperature(self,id = 1):  # 读取温度，共3个选项，1,2,3
        T1 = '01 03 00 66 00 02 24 14'   # 读第一温度，导热油温度
        T2 = '01 03 00 CA 00 02 E4 35'   # 读第二温度，第二传感器温度
        TS = '01 03 02 58 00 02 44 60'   # 读设定温度，设定温度
        T_byte = {'1':T1, '2':T2, '3':TS}
        if id not in [1,2,3]:
            return '温度读取参数不存在，(id = 1，2 or 3)'
        temp_check_byte_stream = bytes.fromhex(T_byte[str(id)])
        Communication.Send_data(self,temp_check_byte_stream)
        back_code = Communication.Recive_data(self)
        # print(unpack('!f', bytes.fromhex('4184cccd'))[0])
        tempera = unpack('>f',back_code[3:7])
        return tempera[0]

    def set_temperature(self,target_temp):  #设置恒温槽的温度，输入一个浮点数
        if target_temp > 110 or target_temp < -70:
            return '温度设置范围超限，-70 ~ 110℃'
        b_stream = hex(unpack('<I', pack('<f', target_temp))[0])
        former_part = '01100258000204' + b_stream[2:]
        crc16_code =  Communication.calc_crc(self,former_part)
        temp_set_byte_stream = bytes.fromhex(former_part + crc16_code)
        Communication.Send_data(self,temp_set_byte_stream)
        back_code = Communication.Recive_data(self)
        # tempera_set = unpack('>f',back_code[4:8])
        if back_code == b'\x01\x10\x02X\x00\x02\xc1\xa3':
            return '温度设置成功'
        return '温度设置失败'

    def Operation_options(self,id):  #设置恒温槽的开闭，0为停止，1为功能全开
        stop = '01 06 02 BC 00 00 49 96'
        circle = '01 06 02 BC 00 01 88 56'
        cool = '01 06 02 BC 00 02 C8 57'
        heat = '01 06 02 BC 00 04 48 55'
        c_c = '01 06 02 BC 00 03 09 97'
        c_h = '01 06 02 BC 00 05 89 95'
        c_c_h = '01 06 02 bc 00 07 08 54'
        options = {'0':stop, '1':c_c_h}
        if id not in [0,1]:
            return '参数不存在(id = 0 or 1)'
        else:
            operation_byte_stream = bytes.fromhex(options[str(id)])
            Communication.Send_data(self,operation_byte_stream)
            back_code = Communication.Recive_data(self)
        return back_code

    def tempera_time(self, tempera_record, time_record):  # input parameter should with filename extension, eg: .txt
        time_step = np.loadtxt(time_record)
        tempera_step = np.loadtxt(tempera_record)
        plt.figure(figsize=(10, 8))
        plt.plot(time_step, tempera_step)
        plt.title('t_t curve')
        plt.xlabel("Time (s)")
        plt.ylabel("Temperature (℃)")
        plt.savefig('Temperature curve' + ".png", bbox_inches='tight')
        plt.clf()
        return '绘图完毕'



# Communication.Print_Used_Com()
# Ret =False #是否创建成功标志
#
# Engine1 = Communication("/dev/ttyUSB0",19200,0.5)
# if (Ret):
#     print(Engine1.set_temperature(10))
#     print(Engine1.check_temperature(1))
#     Engine1.Operation_options(1)
#     #Engine1.Recive_data(1)