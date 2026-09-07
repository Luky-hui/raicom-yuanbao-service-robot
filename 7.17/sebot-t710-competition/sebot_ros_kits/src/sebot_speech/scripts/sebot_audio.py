#!/usr/bin/env python3
import rospy  
from std_msgs.msg import String  
import pygame
import time 
import os


def playAudio(msgAudio):
    """
    语音播报
    audio: msg话题消息
    """
    if msgAudio.data == "naviTimeout":
        return

    if msgAudio.data != None:
        audioPath = "/root/workspace/sebot-t710-competition/sebot_ros_kits/src/sebot_speech/res/audio/match/" + msgAudio.data + ".mp3"

        if os.path.exists(audioPath):  
            pass
        else:  
            audioPath = "/root/workspace/sebot-t710-competition/sebot_ros_kits/src/sebot_speech/res/audio/match/None.mp3"
        if not pygame.mixer.get_init():
            pygame.mixer.init()  # 初始化声音混合器
        pygame.mixer.music.set_volume(1)   # 设置系统音量[0,1]
        pygame.mixer.music.load(audioPath)  # 加载音频文件
        pygame.mixer.music.play()  # 开始播放            
              
if __name__ == '__main__':  
    # 初始化节点  
    rospy.init_node('sebot_audio', anonymous=True)  

    # 创建一个订阅者，订阅名为audio的topic，消息类型为std_msgs/String  
    rospy.Subscriber("/audio", String, playAudio)  

    # spin()函数会保持Python脚本的运行，直到节点被关闭  
    rospy.spin()  
