/**
 * @file joystick.cpp
 * @author HC
 * @brief
 * @version 0.1
 * @date 2023/09/14 16:20:11
 * @copyright  :Copyright (c) 2023
 * @note 具体功能模块:
 */

#include <cstdlib> // c++常用函数类
#include <fcntl.h>
#include <geometry_msgs/Twist.h> // 导入话题发布类
#include <iostream>              // 输入输出类
#include <ros/ros.h>             // ROS类
#include <string>                // 字符串类
#include <sys/stat.h>            // 获取文件属性
#include <sys/types.h>           // 基本系统数据类型
#include <signal.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <fstream>

using namespace std;

#define JS_EVENT_BUTTON 0x01 // 获取按钮状态
#define JS_EVENT_AXIS 0x02   // 获取摇杆状态
#define JS_EVENT_INIT 0x80   // 获取设备模式

std::string jsMode = "BD4A"; // 设置手柄类型（BD4A:北通蝙蝠4 | BD2A:北通蝙蝠2 | SW02:星途2）

int fileJoystick;     // 遥控手柄文件句柄
bool mapping = false; // 初始化建图使能

/**
 * @brief 遥控手柄数据结构体
 *
 */
typedef struct
{
    unsigned int time;    // 手柄触发时间：ms
    short value;          // 操纵值
    unsigned char type;   // 操纵类型：按键/摇杆
    unsigned char number; // 按键编号
} EventJoy;

/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */
void exitSignal(int signum)
{
    close(fileJoystick); // 关闭文件占用
    ROS_INFO_STREAM("Joystick control closed!!!");
    exit(signum);
}

std::string readSysfs(const std::string &path)
{
    std::ifstream f(path);
    std::string value;
    f >> value;
    return value;
}

int main(int argc, char **argv)
{
    float speedLinear = 0.18;  // 直线位移速度m/s
    float speedAngular = 0.25; // 转向速度rad/s

    ros::init(argc, argv, "sebot_joystick");
    ros::NodeHandle rosNode;
    ros::Publisher pubJoystick = rosNode.advertise<geometry_msgs::Twist>("cmd_vel", 10); // cmd_vel话题发布器
    ros::param::get("/sebot_joystick/mapping", mapping);                                 // 接收超声波话题发布使能
    geometry_msgs::Twist cmdMsg;                                                         // 发布的速控话题数据

    // 打开遥控手柄设备文件
    fileJoystick = open("/dev/joystick", O_RDONLY);
    if (fileJoystick < 0)
    {
        ROS_ERROR_STREAM("------- Couldn't open joystick!!! --------");
        return 1;
    }
    // 获取手柄名称
    char sysfs_path[256];
    for (size_t i = 0; i < 5; i++)
    {
        snprintf(sysfs_path, sizeof(sysfs_path), "/sys/class/input/js%d/device/id/", i); // jsX
        std::string vendor = readSysfs(std::string(sysfs_path) + "vendor");
        std::string product = readSysfs(std::string(sysfs_path) + "product");
        std::cout << "---Vendor ID : " << vendor << std::endl;
        std::cout << "---Product ID: " << product << std::endl;
        if (vendor == "045e" && product == "028e") // 北通二代
        {
            jsMode = "BD2A";
            ROS_WARN_STREAM("Joystick selected BD2A...");
            break;
        }
        else if (vendor == "20bc" && product == "5046") // 北通四代
        {
            jsMode = "BD4A";
            ROS_WARN_STREAM("Joystick selected BD4A...");
            break;
        }
        else if (vendor == "3537" && product == "1007") // 星途
        {
            jsMode = "SW02";
            ROS_WARN_STREAM("Joystick selected SW02...");
            break;
        }
    }

    EventJoy joy;   // 实例化遥控手柄结构体
    joy.number = 0; // 初始化按键编号

    bool transmit = false;      // 当数据更新之后才发布话题
    ros::Rate loop_rate(20);    // while循环周期：20Hz/50ms
    signal(SIGINT, exitSignal); // 程序退出信号
    ROS_INFO_STREAM("Joystick remote controling...");
    while (ros::ok())
    {
        read(fileJoystick, &joy, sizeof(joy)); // 读取遥控手柄设备文件
        if (jsMode == "BD2A")
        {
            if (joy.type == JS_EVENT_AXIS) // 如果操纵类型为摇杆
            {
                // ROS_INFO_STREAM("AXIS: [" + to_string(joy.number) + "] = " + to_string(joy.value)); // 打印按键编号和键值
                switch (joy.number)
                {
                case 2: // 旋转低速
                    if (joy.value >= 1)
                    {
                        speedAngular = 0.4; // rad/s
                        transmit = true;    // 需要发布话题
                        ROS_INFO_STREAM("Low speed for turnning:0.4rad/s");
                    }
                    break;
                case 5: // 直线运动低速
                    if (joy.value >= 1)
                    {
                        speedLinear = 0.3; // m/s
                        transmit = true;   // 需要发布话题
                        ROS_INFO_STREAM("Low speed for move:0.3m/s");
                    }
                    break;
                case 6: // 左右平移控制
                    if (!mapping)
                    {
                        if (joy.value < -30000) // 左
                            cmdMsg.linear.y = speedLinear;
                        else if (joy.value > 30000) // 右
                            cmdMsg.linear.y = -speedLinear;
                        else // 停止：按键弹起
                            cmdMsg.linear.y = 0;
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        transmit = true; // 需要发布话题
                    }
                    break;
                }
            }
            else if (joy.type == JS_EVENT_BUTTON) // 按键
            {
                // ROS_INFO_STREAM("BUTTON: [" + to_string(joy.number) + "] = " + to_string(joy.value));
                switch (joy.number)
                {
                case 0:                 // 后退控制
                    if (joy.value >= 1) // 后退
                    {
                        cmdMsg.linear.x = -speedLinear;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 1:                 // 右转
                    if (joy.value >= 1) // 右转
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = -speedAngular;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 2:                 // 左转
                    if (joy.value >= 1) // 左转
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = speedAngular;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 3:                 // 前进控制
                    if (joy.value >= 1) // 前进
                    {
                        cmdMsg.linear.x = speedLinear;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 4: // 旋转高速
                    if (joy.value == 1)
                    {
                        speedAngular = 0.5; // rad/s
                        transmit = true;    // 需要发布话题
                        ROS_INFO_STREAM("High speed for turnning:0.9rad/s");
                    }
                    break;
                case 5: // 直线运动高速
                    if (joy.value >= 1)
                    {
                        speedLinear = 0.3; // m/s
                        transmit = true;   // 需要发布话题
                        ROS_INFO_STREAM("High speed for move:0.5m/s");
                    }
                    break;
                default: // 任意键停止运动
                    cmdMsg.linear.x = 0;
                    cmdMsg.linear.y = 0;
                    cmdMsg.angular.z = 0;
                    transmit = true; // 需要发布话题
                    break;
                }
            }
        }
        else if (jsMode == "BD4A")
        {
            if (joy.type == JS_EVENT_AXIS) // 如果操纵类型为摇杆
            {
                // ROS_INFO_STREAM("AXIS: [" + to_string(joy.number) + "] = " + to_string(joy.value)); // 打印按键编号和键值
                switch (joy.number)
                {
                case 6: // 左右平移控制
                    if (!mapping)
                    {
                        if (joy.value < -30000) // 左
                            cmdMsg.linear.y = speedLinear;
                        else if (joy.value > 30000) // 右
                            cmdMsg.linear.y = -speedLinear;
                        else // 停止：按键弹起
                            cmdMsg.linear.y = 0;
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        transmit = true; // 需要发布话题
                    }
                    break;
                }
            }
            else if (joy.type == JS_EVENT_BUTTON) // 按键
            {
                // ROS_INFO_STREAM("BUTTON: [" + to_string(joy.number) + "] = " + to_string(joy.value));
                switch (joy.number)
                {
                case 0:                 // 后退控制
                    if (joy.value >= 1) // 后退
                    {
                        cmdMsg.linear.x = -speedLinear;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 1:                 // 右转
                    if (joy.value >= 1) // 右转
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = -speedAngular;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 3:                 // 左转
                    if (joy.value >= 1) // 左转
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = speedAngular;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 4:                 // 前进控制
                    if (joy.value >= 1) // 前进
                    {
                        cmdMsg.linear.x = speedLinear;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 6: // 旋转高速
                    if (joy.value == 1)
                    {
                        speedAngular = 0.5; // rad/s
                        transmit = true;    // 需要发布话题
                        ROS_INFO_STREAM("High speed for turnning:0.9rad/s");
                    }
                    break;
                case 7: // 直线运动高速
                    if (joy.value >= 1)
                    {
                        speedLinear = 0.3; // m/s
                        transmit = true;   // 需要发布话题
                        ROS_INFO_STREAM("High speed for move:0.5m/s");
                    }
                    break;
                case 8: // 旋转低速
                    if (joy.value >= 1)
                    {
                        speedAngular = 0.4; // rad/s
                        transmit = true;    // 需要发布话题
                        ROS_INFO_STREAM("Low speed for turnning:0.4rad/s");
                    }
                    break;
                    break;
                case 9: // 直线运动低速
                    if (joy.value >= 1)
                    {
                        speedLinear = 0.3; // m/s
                        transmit = true;   // 需要发布话题
                        ROS_INFO_STREAM("Low speed for move:0.3m/s");
                    }
                    break;
                default: // 任意键停止运动
                    cmdMsg.linear.x = 0;
                    cmdMsg.linear.y = 0;
                    cmdMsg.angular.z = 0;
                    transmit = true; // 需要发布话题
                    break;
                }
            }
        }
        else if (jsMode == "SW02") // 星途
        {
            if (joy.type == JS_EVENT_AXIS) // 如果操纵类型为摇杆
            {
                // ROS_INFO_STREAM("AXIS: [" + to_string(joy.number) + "] = " + to_string(joy.value)); // 打印按键编号和键值
                switch (joy.number)
                {
                case 6: // 左右平移控制
                    if (!mapping)
                    {
                        if (joy.value < -30000) // 左
                            cmdMsg.linear.y = speedLinear;
                        else if (joy.value > 30000) // 右
                            cmdMsg.linear.y = -speedLinear;
                        else // 停止：按键弹起
                            cmdMsg.linear.y = 0;
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        transmit = true; // 需要发布话题
                    }
                    break;
                }
            }
            else if (joy.type == JS_EVENT_BUTTON) // 按键
            {
                // ROS_INFO_STREAM("BUTTON: [" + to_string(joy.number) + "] = " + to_string(joy.value));
                switch (joy.number)
                {
                case 6: // 旋转高速
                    if (joy.value == 1)
                    {
                        speedAngular = 0.5; // rad/s
                        transmit = true;    // 需要发布话题
                        ROS_INFO_STREAM("High speed for turnning:0.9rad/s");
                    }
                    break;
                case 8: // 旋转低速
                    if (joy.value >= 1)
                    {
                        speedAngular = 0.4; // rad/s
                        transmit = true;    // 需要发布话题
                        ROS_INFO_STREAM("Low speed for turnning:0.4rad/s");
                    }
                    break;
                case 4:                 // 前进控制
                    if (joy.value >= 1) // 前进
                    {
                        cmdMsg.linear.x = speedLinear;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 0:                 // 后退控制
                    if (joy.value >= 1) // 后退
                    {
                        cmdMsg.linear.x = -speedLinear;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 1:                 // 右转
                    if (joy.value >= 1) // 右转
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = -speedAngular;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 3:                 // 左转
                    if (joy.value >= 1) // 左转
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = speedAngular;
                        cmdMsg.linear.y = 0;
                    }
                    else // 停止：按键弹起
                    {
                        cmdMsg.linear.x = 0;
                        cmdMsg.angular.z = 0;
                        cmdMsg.linear.y = 0;
                    }
                    transmit = true; // 需要发布话题
                    break;
                case 7: // 直线运动高速
                    if (joy.value >= 1)
                    {
                        speedLinear = 0.3; // m/s
                        transmit = true;   // 需要发布话题
                        ROS_INFO_STREAM("High speed for move:0.5m/s");
                    }
                    break;
                case 9: // 直线运动低速
                    if (joy.value >= 1)
                    {
                        speedLinear = 0.3; // m/s
                        transmit = true;   // 需要发布话题
                        ROS_INFO_STREAM("Low speed for move:0.3m/s");
                    }
                    break;
                default: // 任意键停止运动
                    cmdMsg.linear.x = 0;
                    cmdMsg.linear.y = 0;
                    cmdMsg.angular.z = 0;
                    transmit = true; // 需要发布话题
                    break;
                }
            }
        }

        if (transmit)
        {
            pubJoystick.publish(cmdMsg); // 发布cmd_vel速控话题
            transmit = false;            // 关闭话题发布
        }

        loop_rate.sleep();
    }

    close(fileJoystick);
    return 0;
}
