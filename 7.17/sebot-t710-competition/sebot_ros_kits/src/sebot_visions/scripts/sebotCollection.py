#!/usr/bin/env python
# -*- encoding: utf-8 -*-

'''
@file          :imageCollection.py
@Description   :遥控图像采集
@Date          :2023/10/30 17:54:42
@Autor         :Leo
@Version       :v1.0
'''

import time
import cv2
import sys
sys.path.append(
    "/root/workspace/sebot-t710-competition/sebot_ros_kits/src/sebot_speech/scripts")


if __name__ == "__main__":
    from uart import*
    from joyStick import *
    uart = Uart("/dev/robot")  # 串口初始化
    joy = JoyStick(0, uart)  # 遥控手柄初始化

    time.sleep(1)

    # [1] 相机初始化
    capture = cv2.VideoCapture("/dev/deepCamera")   # 设置摄像头
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置视频流的分辨率为640x480
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    index = 1
    counter = 0
    while True:
        ret,image = capture.read()  # 相机采集是否成功,图像
        if not ret:
            print("摄像头打开失败！请检查摄像头")
            break
        if joy.cotMore or joy.cotOnce:
            path = "{}/{}.jpg".format(
                "/root/workspace/sebot-t710-competition/sebot_ros_kits/src/sebot_visions/res/images/sample", index)
            cv2.imwrite(path, image)

            index += 1
            if joy.cotOnce:
                joy.cotOnce = False

            print("saveimage: {}.jpg".format(index))

        cv2.putText(image, str(index), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 254), 1, cv2.LINE_AA)
        cv2.imshow("image", image)  # UI显示
        if cv2.waitKey(1) == 27:  # 按 ESC 退出程序
            break
        joy.joystickHandle()  # 遥控手柄获取键值,控制机器人速度和图像采集

    uart.stop(0, 0)
