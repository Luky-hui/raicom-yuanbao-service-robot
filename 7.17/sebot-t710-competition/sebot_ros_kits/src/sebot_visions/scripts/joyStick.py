#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@file 		   :joyStick.py
@Description   :遥控手柄 - 支持三种手柄热插拔
@Date 		   :2023/11/11 18:46:21
@Autor         :HC
@Version       :v2.0 - 支持热插拔
'''

import pygame

class JoyStick:
    class Speed:
        def __init__(self):
            self.speedLineX = 0.0  # X轴车速：m/s
            self.speedLineY = 0.0  # Y轴车速：m/s
            self.speedAngle = 0.0  # 旋转速度：rad/s

    def __init__(self, id, uart):
        self.debug = False  # 遥控手柄调试
        self.running = False  # 线程运行标志
        self.cotMore = False  # 连续图像采样使能
        self.cotOnce = False  # 单次图像采样使能
        self.send = False  # 串口发送标志
        self.speedSend = self.Speed()  # 串口发送速度
        self.speedSet = self.Speed()  # 挡位速度
        self.speedSet.speedLineX = 0.3
        self.speedSet.speedAngle = 0.5
        self.uart = uart  # 通信串口类
        
        # 热插拔相关
        pygame.init()
        pygame.joystick.init()
        
        # 存储活跃手柄：{joy_id: JoyStickHandler}
        self.active_joysticks = {}
        
        # 初始化已连接的手柄
        self.initJoysticks()

    def initJoysticks(self):
        """初始化已连接的手柄"""
        for i in range(pygame.joystick.get_count()):
            self.addJoystick(i)

    def addJoystick(self, joy_id):
        """添加一个手柄"""
        try:
            joystick = pygame.joystick.Joystick(joy_id)
            joystick.init()
            name = joystick.get_name()
            guid = joystick.get_guid()
            print(f"🎮 手柄 {joy_id} 已连接: {name} (GUID: {guid})")
            # 识别手柄类型
            joy_handler = JoyStickHandler(joy_id, joystick, guid, self)
            self.active_joysticks[joy_id] = joy_handler
            print(f"✅ 手柄 {joy_id} 类型识别为: {joy_handler.type}")
        except Exception as e:
            print(f"❌ 初始化手柄 {joy_id} 失败: {e}")

    def removeJoystick(self, joy_id):
        """移除一个手柄"""
        if joy_id in self.active_joysticks:
            handler = self.active_joysticks[joy_id]
            print(f"⏏️ 手柄 {joy_id} ({handler.type}) 已断开")
            handler.joyStick.quit()
            del self.active_joysticks[joy_id]

    def joystickHandle(self):
        """
        遥控手柄获取键值,控制机器人速度和图像采集（支持热插拔）
        """
        # 处理事件队列
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            # 手柄插入
            elif event.type == pygame.JOYDEVICEADDED:
                joy_id = event.device_index
                self.addJoystick(joy_id)
                print("✅手柄已插入...")

            # 手柄拔出
            elif event.type == pygame.JOYDEVICEREMOVED:
                print("[WARN] 手柄已拔出!!!")
                try:
                    joy_id = event.joy  # 拔出时使用 joy
                    self.removeJoystick(joy_id)
                except AttributeError:
                    # 尝试重新构建活跃手柄列表，移除无效手柄
                    invalid_ids = []
                    for joy_id, handler in self.active_joysticks.items():
                        try:
                            # 尝试获取手柄状态，如果失败则认为手柄已断开
                            _ = handler.joyStick.get_numbuttons()
                        except pygame.error:
                            invalid_ids.append(joy_id)
                    for joy_id in invalid_ids:
                        self.removeJoystick(joy_id)
                except Exception as e:
                    print(f"⚠️ 处理手柄拔出事件失败: {e}")

            # 将事件分发给对应的手柄处理器
            elif event.type in [pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP, 
                               pygame.JOYAXISMOTION, pygame.JOYHATMOTION]:
                if event.joy in self.active_joysticks:
                    self.active_joysticks[event.joy].handle_event(event)

        # 检查是否有待发送的速度指令
        if self.send:
            self.uart.speedControl(
                self.speedSend.speedLineX, 
                self.speedSend.speedLineY, 
                self.speedSend.speedAngle
            )
            self.send = False

        # 发送心跳信号
        self.uart.transmitSysHeart()


class JoyStickHandler:
    """单个手柄的处理器"""
    def __init__(self, joy_id, joystick, guid, parent):
        self.joy_id = joy_id
        self.joyStick = joystick
        self.guid = guid
        self.parent = parent
        
        # 识别手柄类型
        self.type = self.getTypes()
        
        # 按键状态
        self.numBtn = self.joyStick.get_numbuttons()
        self.numAxes = self.joyStick.get_numaxes()
        self.numhat = self.joyStick.get_numhats()
        self.staBtn = [False] * self.numBtn

    def getTypes(self):
        """根据GUID识别手柄类型"""
        if self.guid == "030003f05e0400008e02000011010000":  # 北通二代
            return "BD2A"
        elif self.guid == "03004725bc2000004650000011010000":  # 北通四代
            return "BD4A"
        elif self.guid == "030017c8373500000710000010010000":  # 星途2
            return "SW02"
        else:
            print(f"⚠️ 未知手柄 GUID: {self.guid}")
            return "UNKNOWN"

    def handle_event(self, event):
        """处理单个手柄的事件"""
        if self.type == "BD2A":
            self.joyHandleBD2A(event)
        elif self.type == "BD4A":
            self.joyHandleBD4A(event)
        elif self.type == "SW02":
            self.joyHandleBD4A(event)  # 星途2与BD4A按键映射相同

    def joyHandleBD2A(self, event):
        """
        北通蝙蝠2手柄键值处理
        """
        if event.type == pygame.JOYBUTTONUP or event.type == pygame.JOYBUTTONDOWN:
            for i in range(self.numBtn):
                if self.joyStick.get_button(i):  # 按键按下
                    if not self.staBtn[i]:
                        if self.parent.debug:
                            print(f"BD2A button-down[{i}]")

                        if i == 3:  # 前进
                            self.parent.speedSend.speedLineX = self.parent.speedSet.speedLineX
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = 0.0
                            self.parent.send = True
                        elif i == 0:  # 后退
                            self.parent.speedSend.speedLineX = -self.parent.speedSet.speedLineX
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = 0.0
                            self.parent.send = True
                        elif i == 2:  # 左转
                            self.parent.speedSend.speedLineX = 0.0
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = self.parent.speedSet.speedAngle
                            self.parent.send = True
                        elif i == 1:  # 右转
                            self.parent.speedSend.speedLineX = 0.0
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = -self.parent.speedSet.speedAngle
                            self.parent.send = True
                        elif i == 4:  # 旋转高速
                            self.parent.speedSet.speedAngle = 0.9
                            print("High speed for turnning:0.9rad/s")
                        elif i == 5:  # 直线运动高速
                            self.parent.speedSet.speedLineX = 0.5
                            print("High speed for move:0.5m/s")
                        elif i == 9:  # 单帧采图
                            self.parent.cotOnce = True
                        elif i == 10:  # 停止采图
                            self.parent.cotMore = False
                            self.parent.cotOnce = False
                        else:  # 任意键停止运动
                            self.parent.speedSend.speedLineX = 0.0
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = 0.0
                            self.parent.send = True

                    self.staBtn[i] = True
                else:
                    if self.staBtn[i]:  # 按键弹起
                        if self.parent.debug:
                            print(f"BD2A button-up[{i}]")

                        if i in [0, 1, 2, 3]:  # 前后左右键弹起时停止运动
                            self.parent.speedSend.speedLineX = 0.0
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = 0.0
                            self.parent.send = True

                    self.staBtn[i] = False

        elif event.type == pygame.JOYAXISMOTION:
            if self.parent.debug:
                print("BD2A axes:", [round(self.joyStick.get_axis(j), 3) for j in range(self.numAxes)])

            if self.joyStick.get_axis(2) > 0:  # 旋转低速
                self.parent.speedSet.speedAngle = 0.5
                print("Low speed for turnning:0.5rad/s")
            elif self.joyStick.get_axis(5) > 0:  # 直线运动低速
                self.parent.speedSet.speedLineX = 0.3
                print("Low speed for move:0.3m/s")

        elif event.type == pygame.JOYHATMOTION and self.numhat > 0:
            x, y = self.joyStick.get_hat(0)  # 获取键帽值
            if x < 0:  # 左平移
                self.parent.speedSend.speedLineX = 0.0
                self.parent.speedSend.speedLineY = self.parent.speedSet.speedLineX
                self.parent.speedSend.speedAngle = 0.0
                self.parent.send = True
            elif x > 0:  # 右平移
                self.parent.speedSend.speedLineX = 0.0
                self.parent.speedSend.speedLineY = -self.parent.speedSet.speedLineX
                self.parent.speedSend.speedAngle = 0.0
                self.parent.send = True
            elif x == 0:  # 停止：按键弹起
                self.parent.speedSend.speedLineX = 0.0
                self.parent.speedSend.speedLineY = 0.0
                self.parent.speedSend.speedAngle = 0.0
                self.parent.send = True

    def joyHandleBD4A(self, event):
        """
        北通蝙蝠4/星途2手柄键值处理
        """
        if event.type == pygame.JOYBUTTONUP or event.type == pygame.JOYBUTTONDOWN:
            for i in range(self.numBtn):
                if self.joyStick.get_button(i):  # 按键按下
                    if not self.staBtn[i]:
                        if self.parent.debug:
                            print(f"{self.type} button-down[{i}]")

                        if i == 4:  # 前进：Y
                            self.parent.speedSend.speedLineX = self.parent.speedSet.speedLineX
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = 0.0
                            self.parent.send = True
                        elif i == 0:  # 后退: A
                            self.parent.speedSend.speedLineX = -self.parent.speedSet.speedLineX
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = 0.0
                            self.parent.send = True
                        elif i == 3:  # 左转: X
                            self.parent.speedSend.speedLineX = 0.0
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = self.parent.speedSet.speedAngle
                            self.parent.send = True
                        elif i == 1:  # 右转: B
                            self.parent.speedSend.speedLineX = 0.0
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = -self.parent.speedSet.speedAngle
                            self.parent.send = True
                        elif i == 6:  # 旋转高速: LB
                            self.parent.speedSet.speedAngle = 0.9
                            print("High speed for turnning:0.9rad/s")
                        elif i == 7:  # 直线运动高速: RB
                            self.parent.speedSet.speedLineX = 0.5
                            print("High speed for move:0.5m/s")
                        elif i == 13:  # 单帧采图: 左侧摇杆按键
                            self.parent.cotOnce = True
                        elif i == 14:  # 停止采图：右侧摇杆按键
                            self.parent.cotMore = False
                            self.parent.cotOnce = False
                        else:  # 任意键停止运动
                            self.parent.speedSend.speedLineX = 0.0
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = 0.0
                            self.parent.send = True

                    self.staBtn[i] = True
                else:
                    if self.staBtn[i]:  # 按键弹起
                        if self.parent.debug:
                            print(f"{self.type} button-up[{i}]")

                        if i in [0, 1, 3, 4]:  # 前后左右键弹起时停止运动
                            self.parent.speedSend.speedLineX = 0.0
                            self.parent.speedSend.speedLineY = 0.0
                            self.parent.speedSend.speedAngle = 0.0
                            self.parent.send = True

                    self.staBtn[i] = False

        elif event.type == pygame.JOYAXISMOTION:
            if self.parent.debug:
                print(f"{self.type} axes:", [round(self.joyStick.get_axis(j), 3) for j in range(self.numAxes)])

            if self.joyStick.get_axis(5) > 0:  # 旋转低速：LT
                self.parent.speedSet.speedAngle = 0.5
                print("Low speed for turnning:0.5rad/s")
            elif self.joyStick.get_axis(4) > 0:  # 直线运动低速：RT
                self.parent.speedSet.speedLineX = 0.3
                print("Low speed for move:0.3m/s")

        elif event.type == pygame.JOYHATMOTION and self.numhat > 0:
            x, y = self.joyStick.get_hat(0)  # 获取键帽值
            if x < 0:  # 左平移
                self.parent.speedSend.speedLineX = 0.0
                self.parent.speedSend.speedLineY = self.parent.speedSet.speedLineX
                self.parent.speedSend.speedAngle = 0.0
                self.parent.send = True
            elif x > 0:  # 右平移
                self.parent.speedSend.speedLineX = 0.0
                self.parent.speedSend.speedLineY = -self.parent.speedSet.speedLineX
                self.parent.speedSend.speedAngle = 0.0
                self.parent.send = True
            elif x == 0:  # 停止：按键弹起
                self.parent.speedSend.speedLineX = 0.0
                self.parent.speedSend.speedLineY = 0.0
                self.parent.speedSend.speedAngle = 0.0
                self.parent.send = True


if __name__ == "__main__":
    import sys
    from time import sleep
    sys.path.append(
        "/root/workspace/sebot-t710-competition/sebot_ros_kits/src/sebot_speech/scripts")
    
    from uart import*
    uart = Uart("/dev/ttyUSB0")  # 串口初始化
    joy = JoyStick(0, uart)  # 遥控手柄初始化
    
    print("🎮 手柄系统启动，支持热插拔！")
    print("连接BD4A/BD2A/SW02手柄，系统将自动识别并处理控制信号")
    
    while True:
        sleep(0.05)
        joy.joystickHandle()  # 遥控手柄获取键值,控制机器人速度和图像采集