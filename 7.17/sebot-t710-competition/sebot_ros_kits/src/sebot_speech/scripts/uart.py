#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@file          :uart.py
@Description   :串口通信(上下位机),协议编解码校验
@Date          :2023/07/12 14:38:53
@Autor         :Leo
@Version       :v1.0
"""
import threading
import serial
import time
import struct
import signal

# 通信地址定义
USB_FRAME_HEAD = bytes.fromhex("42")  # 通信序列帧头
USB_FRAME_LENMAX = 36  # 通信序列字节最长长度
USB_FRAME_LENMIN = 4  # 通信序列字节最短长度

# 通信地址
USB_ADDR_SPEEDCTRL = bytes.fromhex("01")  # 速度控制
USB_ADDR_BATTERY = bytes.fromhex("02")  # 电量数据 :1s
USB_ADDR_ULTRA = bytes.fromhex("03")  # 超声波数据 :100ms
USB_ADDR_ODOM = bytes.fromhex("04")  # 里程数据 :30ms
USB_ADDR_IMU = bytes.fromhex("05")  # IMU数据 :30ms
USB_ADDR_MOVECTRL = bytes.fromhex("06")  # 运动控制
USB_ADDR_BEEP = bytes.fromhex("07")  # 蜂鸣器
USB_ADDR_ERRORCODE = bytes.fromhex("08")  # 错误代码
USB_ADDR_LIDAR = bytes.fromhex("09")  # 激光雷达使能
USB_ADDR_CHECK = bytes.fromhex("0A")  # 机器人自检
USB_ADDR_RGB = bytes.fromhex("0B")  # 氛围灯控制

USB_ADDR_HART = bytes.fromhex("37")  # 心跳信号
USB_ADDR_RESET = bytes.fromhex("42")  # 系统复位
USB_ADDR_REBOOT = bytes.fromhex("4D")  # 下位机重启


class Uart:
    """
    定义串口通信类
    """

    def __init__(self, port):
        """
        初始化串口
        """
        self.port = port  # 串口名称
        self.baudRate = 115200  # 波特率
        self.serial = None  # 初始化串口对象
        self.thread = None  # 串口线程
        self.recStart = False  # 数据接收开始标志
        self.recIndex = 0  # 接收序列号
        self.len = 0  # 帧长
        self.recBuff = [0] * USB_FRAME_LENMAX  # 临时接收数据区
        self.timeHart = time.time()  # 记录心跳信号发送时间：s(float)
        self.connect = False  # 通信连接标志
        self.timeSpeedCtrl = time.time()  # 记录速度控制时间
        self.sensors = self.Sensors()  # 传感器数据
        self.running = False
        self.serial = serial.Serial(
            self.port,
            int(self.baudRate),
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=1,
        )

    class Sensors:
        """
        传感器数据
        """
        voltage = 0.0  # 电池电压：0~30.0v
        power = 0  # 电池电量0~100%
        ultra = [0.0]*4  # 超声波距离: 0~1m
        errorCode = 0  # 下位机错位代码0~65535

    def start(self,type=False):
        """
        开启串口通信接收子线程
        """
        if type:
            self.serial = serial.Serial(
                self.port,
                int(self.baudRate),
                timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=1,
            )
        # 创建子线程并启动
        self.thread = threading.Thread(target=self.serialThread)
        self.running = True
        print("[Info] uart receive thread start!")
        self.thread.start()

    def stop(self, signum, frame):
        """
        停止进程
        """
        self.speedControl(0.0, 0.0, 0.0)  # 停止运动
        self.running = False
        time.sleep(0.2)
        # 关闭串口
        if self.serial:
            self.serial.close()
        print("[Info] uart thread is closed!")

    def serialThread(self):
        """
        串口接收多线程
        """
        time.sleep(0.5)
        # 判断串口是否运行
        while self.running:
            # 判断串口是否可读
            if self.serial.in_waiting > 0:
                data = self.serial.read(1)
                # 如果接收到的字节是帧头且串口没有开始接收
                if data == USB_FRAME_HEAD and not self.recStart:
                    self.recStart = True
                    self.recBuff[0] = data
                    self.recBuff[2] = USB_FRAME_LENMIN
                    self.recIndex = 1
                elif self.recIndex == 2:
                    self.recBuff[2] = data
                    self.recIndex += 1
                    self.len = int.from_bytes(
                        data, byteorder="big", signed=False)
                    if self.len > USB_FRAME_LENMAX or self.len < USB_FRAME_LENMIN:  # 判断帧长是否合理
                        self.recBuff[2] = USB_FRAME_LENMIN
                        self.recIndex = 0
                        self.recStart = False
                elif self.recStart and self.recIndex < USB_FRAME_LENMAX:  # 连续接收数据帧
                    if data == USB_FRAME_HEAD and self.recIndex == 1:
                        self.recIndex = 1
                    else:
                        self.recBuff[self.recIndex] = data
                        self.recIndex += 1

                if (
                    self.recIndex >= USB_FRAME_LENMAX or self.recIndex >= self.len
                ) and self.recIndex > USB_FRAME_LENMIN:
                    check = int(0)  # 数据校验

                    for i in range(self.len - 1):  # 校验和（Byte）
                        check += int.from_bytes(self.recBuff[i],
                                                byteorder="big", signed=False)

                    check = int(check % 256)
                    if check == int.from_bytes(
                        self.recBuff[self.len - 1], byteorder="big", signed=False
                    ):  # 数据校验
                        self.recIndex = 0
                        self.recStart = False
                        self.recLength = USB_FRAME_LENMAX
                        self.timeDrop = time.time()  # 刷新通信掉线时间
                        self.connect = True  # 通信连接成功标志
                        self.transDataFrame()  # 接收成功，数据回响

    def transDataFrame(self):
        """
        串口通信帧数据解析,C语言与Python转换标准:
        ----------------------------------------------------------------
        字符    C类型	                Python类型	           标准大小
        x	    填充字节	            无对应值	             []3
        c	    char	                长度为1的bytes	         1
        b	    signed char	            integer	                1
        B	    unsigned char	        integer	                1
        ?	    _Bool	                bool	                1
        h	    short	                integer	                2
        H	    unsigned char	        integer	                2
        i	    int	                    integer	                4
        I	    unsigned int	        integer	                4
        l	    long	                integer	                4
        L	    unsigned long	        integer	                4
        q	    long long	            nteger	                8
        Q	    unsigned long long      integer	                8
        n	    ssize_t	                integer	                []4
        N	    size_t	                integer	                []4
        e	    []5	                    float	                2
        f	    float	                float	                4
        d	    double	                float	                8
        s	    char[]	                bytes	                []6
        p	    char[]	                bytes	                []7
        P	    void*	                integer	                []8
        -----------------------------------------------------------------
        """
        if self.recBuff[1] == USB_ADDR_ERRORCODE:   # 错误代码
            self.sensors.errorCode = struct.unpack(
                "H", self.recBuff[3] + self.recBuff[4],)[0]
        elif self.recBuff[1] == USB_ADDR_BATTERY:  # 电池信息
            self.sensors.power = int.from_bytes(
                self.recBuff[3], byteorder="big", signed=False)  # 电量
            self.sensors.voltage = struct.unpack(
                "f", self.recBuff[4] + self.recBuff[5] + self.recBuff[6] + self.recBuff[7],)[0]  # 电池电压
        elif self.recBuff[1] == USB_ADDR_ULTRA:  # 超声波数据
            for i in range(4):
                self.sensors.ultra[i] = struct.unpack(
                    "f", self.recBuff[3+4*i] + self.recBuff[4+4*i] + self.recBuff[5+4*i] + self.recBuff[6+4*i],)[0]

    def transmitFrame(self, addr, data):
        """
        串口发送帧数据转换
        addr: 通信地址
        data: 数据流
        """
        # 判断串口是否运行
        if self.serial.is_open == False:
            return

        if data == None:
            frame = (
                USB_FRAME_HEAD  # 帧头
                + addr  # 地址
                + (4).to_bytes(1, byteorder="little", signed=False))  # 帧长
        elif len(data) > 0:
            frame = (
                USB_FRAME_HEAD  # 帧头
                + addr  # 地址
                + (len(data) + 4).to_bytes(1,
                                           byteorder="little", signed=False)  # 帧长
                + data  # 数据
            )

        check = int(0)
        for i in frame:
            check += i
            check = int(check % 256)
        frame = frame + check.to_bytes(1, byteorder="little", signed=False)
        self.serial.write(frame + bytes.fromhex("00"))
        self.timeHart = time.time()  # 获取系统时间：s(float)

    def speedControl(self, speedlineX, speedlineY, speedAngle):
        """
        机器人速度控制
        speedlineX: X轴线速度: m/s
        speedlineY: Y轴线速度: m/s
        speedAngle: Z轴角速度: rad
        """
        thisTime = time.time()  # 获取系统时间：s(float)
        if thisTime - self.timeSpeedCtrl > 0.05:  # 自动50ms发送一次心跳信号
            self.timeSpeedCtrl = thisTime
            stream = struct.pack(
                "<f", speedlineX) + struct.pack("<f", speedlineY) + struct.pack("<f", speedAngle)
            self.transmitFrame(USB_ADDR_SPEEDCTRL, stream)

    def transmitSysHeart(self):
        """
        发送系统心跳(下位机掉线检测时间5s, 掉线后不主动上传数据)
        """
        thisTime = time.time()  # 获取系统时间：s(float)
        if thisTime - self.timeHart > 1:  # 自动1s发送一次心跳信号
            self.transmitFrame(USB_ADDR_HART, None)
            self.timeHart = thisTime

    def systemReset(self):
        """
        系统复位(灯光等外设复位)
        """
        self.transmitFrame(USB_ADDR_RESET, None)

    def movement2X(self, speed, distance):
        """
        里程计闭环位移控制(X轴)
        speed: 运动速度 >0m/s
        distance: 运动距离m, 前>0, 后<0
        """
        if speed > 0:
            stream = bytes.fromhex("01")  # 运动模式
            stream += struct.pack("<f", speed) + struct.pack("<f", distance)
            self.transmitFrame(USB_ADDR_MOVECTRL, stream)

    def movement2Y(self, speed, distance):
        """
        里程计闭环位移控制(X轴)
        speed: 运动速度 >0m/s
        distance: 运动距离m, 左>0, 右<0
        """
        if speed > 0:
            stream = bytes.fromhex("02")  # 运动模式
            stream += struct.pack("<f", speed) + struct.pack("<f", distance)
            self.transmitFrame(USB_ADDR_MOVECTRL, stream)

    def turnning(self, speed, angle):
        """
        里程计闭环转向控制(X轴)
        speed: 运动速度 >rad/s
        angle: 转向角度rad, 左>0, 右<0
        """
        if speed > 0:
            stream = bytes.fromhex("03")  # 运动模式
            stream += struct.pack("<f", speed) + struct.pack("<f", angle)
            self.transmitFrame(USB_ADDR_MOVECTRL, stream)

    def setBeepSound(self, sound):
        """
        蜂鸣器音效
        sound: 音效(1/2/3/4/5: 确认/报警/完成/提示/开机)
        """
        if sound > 0 and sound <= 5:
            self.transmitFrame(USB_ADDR_BEEP, struct.pack("<B", sound))

    def lidarEnable(self, enable):
        """
        激光雷达启/停
        enable: True/False 开/关
        """
        if enable:
            self.transmitFrame(USB_ADDR_LIDAR, struct.pack("<B", 1))
        else:
            self.transmitFrame(USB_ADDR_LIDAR, struct.pack("<B", 2))

    def systemReboot(self):
        """
        下位机MCU重启
        """
        self.transmitFrame(USB_ADDR_REBOOT, None)

    def robotSelfcheck(self):
        """
        机器人自检（电机）
        """
        self.transmitFrame(USB_ADDR_CHECK, None)

    def rgbEnable(self, enable):
        """
        RGB灯控制: 开/关
        enable: True/False 开/关
        """
        if enable:
            self.transmitFrame(USB_ADDR_RGB, struct.pack("<B", 1))
        else:
            self.transmitFrame(USB_ADDR_RGB, struct.pack("<B", 2))

"""
串口通信测试
"""
if __name__ == "__main__":
    # 串口通信
    uart = Uart("/dev/robot")
    signal.signal(signal.SIGINT, uart.stop)  # 定义软件退出信号量
    uart.start()
    uart.transmitSysHeart()  # 发送心跳信号
    time.sleep(0.2)
    uart.turnning(0.5, -1)
    while uart.running:
        uart.transmitSysHeart()  # 发送心跳信号
        time.sleep(0.2)

    uart.stop(0, 0)
