/**
 * @file keysControl.cpp
 * @author HC
 * @brief 键盘操控机器人运动
 * @version 0.1
 * @date 2023/09/14 11:29:02
 * @copyright  :Copyright (c) 2023
 * @note 具体功能模块:
 */

#include <geometry_msgs/Twist.h> // 导入话题发布类
#include <iostream>              // 输入输出类
#include <ros/ros.h>             // ROS类
#include <termios.h>             // 在POSIX规范中定义的标准接口结构体
#include <unistd.h>              // 系统类

using namespace std;

/**
 * @brief 键盘输出
 *
 * @return int
 */
int getchKeyboard(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // 存储旧设置，并复制到新设置
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // 进行必要的更改并应用设置
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // 获取当前字符
    ch = getchar();

    // 重新应用旧设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main(int argc, char **argv)
{
    float speedLinear = 0.18;  // 直线位移速度
    float speedAngular = 0.25; // 转向速度

    // ROS节点初始化
    ros::init(argc, argv, "sebot_keyboard");
    ros::NodeHandle rosNode;
    ros::Publisher pubKeyboard = rosNode.advertise<geometry_msgs::Twist>("cmd_vel", 10); // cmd_vel话题发布器

    // cmd_vel话题发布器
    geometry_msgs::Twist cmdMsg; // 发布的速控话题数据
    ROS_INFO_STREAM("Keyboard remote controling...");

    while (ros::ok())
    {
        char key(' ');
        key = getchKeyboard(); // 获取键盘输入键值
        // ROS_INFO_STREAM("KeyInput: " + to_string(key));
        switch (key)
        {
        case 119: // W | 向前
            cmdMsg.linear.x = speedLinear;
            cmdMsg.linear.y = 0;
            cmdMsg.angular.z = 0;
            break;
        case 115: // S | 向后
            cmdMsg.linear.x = -speedLinear;
            cmdMsg.linear.y = 0;
            cmdMsg.angular.z = 0;
            break;
        case 97: // A | 向左转
            cmdMsg.linear.x = 0;
            cmdMsg.linear.y = 0;
            cmdMsg.angular.z = speedAngular;
            break;
        case 100: // D | 向右转
            cmdMsg.linear.x = 0;
            cmdMsg.linear.y = 0;
            cmdMsg.angular.z = -speedAngular;
            break;
        case 113: // Q | 左横向（麦轮）
            cmdMsg.linear.x = 0;
            cmdMsg.linear.y = speedLinear;
            cmdMsg.angular.z = 0;
            break;
        case 101: // E | 右横向（麦轮）
            cmdMsg.linear.x = 0;
            cmdMsg.linear.y = -speedLinear;
            cmdMsg.angular.z = 0;
            break;
        case 117: // (U) 旋转高速rad/s
            speedAngular = 0.4;
            break;
        case 106: // (J) 旋转低速rad/s
            speedAngular = 0.4;
            break;
        case 105: // (I) 移动高速：ms/s
            speedLinear = 0.3;
            break;
        case 107: // (K) 移动低速：ms/s
            speedLinear = 0.3;
            break;
        case 111: // (O) 单帧采图
            break;
        case 112: // (P) 连续采图
            break;
        case 108: // (L) 停止采图
            break;

        default: // 停止
            cmdMsg.linear.x = 0;
            cmdMsg.linear.y = 0;
            cmdMsg.angular.z = 0;
            break;
        }

        if (key == 3) // Ctrl+C | 退出
        {
            printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                SASU-SEBOT\n\n");
            break;
        }
        pubKeyboard.publish(cmdMsg); // 发布cmd_vel速控话题
        ros::spinOnce();
    }

    return 0;
}
