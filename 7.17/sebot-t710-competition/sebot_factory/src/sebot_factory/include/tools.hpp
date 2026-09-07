#pragma once
/**
 * @file tools.hpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 基础公共类方法
 * @version 0.1
 * @date 2024-03-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <sys/time.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include "camera.hpp"

using namespace std;

#define COLSIMAGE 320            // 图像的列数
#define ROWSIMAGE 240            // 图像的行数
#define LABEL_AI_NUT    "nut"           // AI标签：螺母
#define LABEL_AI_SCREW  "screw"         // AI标签：螺钉
#define LABEL_AI_PCB    "pcb"           // AI标签：电路板
#define LABEL_AI_BLOCK  "block"         // AI标签：端子排
#define LABEL_AI_TAPE   "tape"          // AI标签：绝缘胶带
#define LABEL_AI_ORDER  "order"         // AI标签：订单牌

/**
 * @brief 获取系统当前时间戳:ms
 *
 * @return long
 */
long getSystTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/**
 * @brief 语音播报话题通信
 *
 * @param pubAudio 话题发布器
 * @param audio 音频文件名称 “factory”
 */
void publishAudio(ros::Publisher pubAudio, string audio)
{
    std_msgs::String msgAudio;
    msgAudio.data = audio;
    pubAudio.publish(msgAudio); // 发布语音播报话题
}

/**
 * @brief    PID姿态控制器
 **/
class Pid
{
public:
    bool enable = false;    // 控制器使能
    float ref = 0.0;        // 姿态PID：期望
    float feedBack = 0.0;   // 姿态PID：反馈
    float preError = 0.0;   // 姿态PID：误差
    float preDerror = 0.0;  // 姿态PID：误差速度
    float kP = 1;           // 姿态PID：比例系数
    float kI = 0.2;         // 姿态PID：积分系数
    float kD = 0.06;        // 姿态PID：微分系数
    float out = 0.0;        // 姿态PID：输出
    float outMax = 0.3;     // 姿态PID：输出最值(姿态差速/rad | 速度m/s)
    uint16_t counter = 0.0; // 计数器
    float deadline = 0.04;  // 死区
};

/**
 * @brief 姿态PID控制器
 *
 * @param pid
 */
void pidController(Pid &pid)
{
    float error, derror, dderror;

    error = pid.ref - pid.feedBack;
    derror = error - pid.preError;
    dderror = derror - pid.preDerror;

    pid.preError = error;
    pid.preDerror = derror;

    if (abs(error) <= pid.deadline) // PID死区限制
        pid.counter++;
    else
        pid.counter = 0;

    if (pid.counter < 10) // 500ms 防止积分饱和
        pid.out += (pid.kP * derror + pid.kI * error + pid.kD * dderror);
    else
        pid.out = 0;

    if (pid.out >= pid.outMax)
        pid.out = pid.outMax;
    else if (pid.out <= -pid.outMax)
        pid.out = -pid.outMax;
}

/**
 * @brief 姿态PD控制器
 *
 * @param controlCenter 智能车控制中心
 */
void pdController(Pid &pid)
{
    // float error = pid.ref - pid.feedBack; // 图像控制中心转换偏差
    // if (abs(error - pid.preError) > COLSIMAGE / 10)
    // {
    //     error = error > pid.preError ? pid.preError + COLSIMAGE / 10 : pid.preError - COLSIMAGE / 10;
    // }

    // pid.out = (error * pid.kP) + (error - pid.preError) * pid.kD;
    // pid.preError = error;

    // if (pid.out >= pid.outMax)
    //     pid.out = pid.outMax;
    // else if (pid.out <= -pid.outMax)
    //     pid.out = -pid.outMax;

    float error, derror, dderror;

    error = pid.ref - pid.feedBack;
    derror = error - pid.preError;
    dderror = derror - pid.preDerror;

    pid.preError = error;
    pid.preDerror = derror;

    if (abs(error) > pid.deadline) // PID死区限制
        pid.out += (pid.kP * derror + pid.kI * error + pid.kD * dderror);

    if (pid.out >= pid.outMax)
        pid.out = pid.outMax;
    else if (pid.out <= -pid.outMax)
        pid.out = -pid.outMax;
}
