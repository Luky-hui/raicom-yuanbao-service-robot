/**
 * @file motionController.cpp
 * @author HC
 * @brief 机器人上下位机通信/运动控制
 * @version 0.1
 * @date 2023-09-13
 *
 * @copyright Copyright (c) 2023
 * @note 具体功能模块：
 *                  [1]接收来自键盘/手柄的数据，并通过串口通信控制机器人运动；
 *                  [2]解析串口接收回来的里程计和超声波数据并根据参数设定来决定是否发布相应的话题。
 */

#include "../include/uart.hpp"        // 串口通信类
#include "ros/ros.h"                  // ROS类
#include "ros/time.h"                 // ROS时钟类
#include "sensor_msgs/Imu.h"          // imu数据类型
#include "sensor_msgs/Range.h"        // 传感器数据类型
#include "std_msgs/String.h"          // 字符类型数据类型
#include <geometry_msgs/Twist.h>      // 导入话题发布类
#include <nav_msgs/Odometry.h>        // 里程计数据
#include <tf/transform_broadcaster.h> // 机器人坐标系
#include <unistd.h>                   // 系统类
#include <signal.h>

using namespace std;

enum UltraEnum // 超声波数据枚举
{
    ULTRA_LF = 0, // 左前
    ULTRA_MF,     // 中前
    ULTRA_RF,     // 右前
    ULTRA_MB      // 中后
};

/**
 * @brief 机器人速度控制
 *
 */
typedef struct
{
    float speedLineX; // X轴速度: m/s
    float speedLineY; // X轴速度: m/s
    float speedAngle; // 角速度 rad/s
    int counter;      // 发送计数
} SpeedSetting;

std::shared_ptr<Uart> uart = nullptr;               // 初始化串口驱动
bool odomEnable = false;                            // 初始化里程计发布使能
bool ultraEnale = false;                            // 初始化超声波发布使能
bool imuEnable = false;                             // 初始化imu里程计发布使能
ros::Publisher pubUltraLF;                          // 超声波话题发布器（左前）
ros::Publisher pubUltraMF;                          // 超声波话题发布器（中前）
ros::Publisher pubUltraRF;                          // 超声波话题发布器（右前）
ros::Publisher pubUltraMB;                          // 超声波话题发布器（中后）
ros::Publisher pubOdom;                             // 里程计话题发布器
ros::Publisher pubImu;                              // imu里程计话题发布器
void speedControl(const geometry_msgs::Twist &cmd); // 机器人速度话题cmd接收回响函数
void exitSignal(int signum);                        // 系统信号回调函数：系统退出
void topicPublishOdom(void);
void topicPublishImu(void);
void topicPublishUltra(UltraEnum ultra);
SpeedSetting speedSetting; // 机器人速度控制

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "sebot_controller"); // 设定节点名称
    ros::NodeHandle rosNode;                   // 初始化节点资源

    // 串口初始化
    // USB转串口的设备名为 / dev/ttyUSB0
    uart = std::make_shared<Uart>("/dev/robot", BaudRate::BAUD_115200);
    if (uart == nullptr)
    {
        ROS_ERROR_STREAM("Create Uart-Driver Error!");
        return -1;
    }
    // 串口初始化，打开串口设备及配置串口数据格式
    int ret = uart->open();
    if (ret != 0)
    {
        ROS_ERROR_STREAM("Uart Open failed!");
        return -1;
    }
    uart->receiveThread(); // 开启串口接收子线程
    sleep(2);              // 等待串口初始化准备

    ros::param::get("/sebot_controller/ultraEnale", ultraEnale); // 接收超声波话题发布使能
    ros::param::get("/sebot_controller/odomEnable", odomEnable); // 接收里程计话题发布使能
    ros::param::get("/sebot_controller/imuEnable", imuEnable);   // 接收Imu话题发布使能

    ros::Subscriber subCmd = rosNode.subscribe("cmd_vel", 30, speedControl); // 订阅cmd_vel话题

    if (ultraEnale)
    {
        pubUltraLF = rosNode.advertise<sensor_msgs::Range>("msgUltraLF", 1); // 定义要发布的超声波话题
        pubUltraMF = rosNode.advertise<sensor_msgs::Range>("msgUltraMF", 1);
        pubUltraRF = rosNode.advertise<sensor_msgs::Range>("msgUltraRF", 1);
        pubUltraMB = rosNode.advertise<sensor_msgs::Range>("msgUltraMB", 1);
    }
    if (odomEnable)
        pubOdom = rosNode.advertise<nav_msgs::Odometry>("odom", 30); // 定义要发布的/odom话题
    if (imuEnable)
        pubImu = rosNode.advertise<sensor_msgs::Imu>("imu", 30); // 定义要发布的/imu话题

    ros::Rate loop_rate(30); // 设置ROS循环周期: Hz

    signal(SIGINT, exitSignal); // 程序退出信号
    ROS_INFO_STREAM("------sebot_controller working!------");
    uart->transmitReset(); // 发送复位信号
    sleep(1);
    uart->lidarEnable(true); // 激光雷达使能

    while (ros::ok()) // 33.3ms
    {
        // 传感器数据发布
        if (odomEnable)
            topicPublishOdom(); // 发布里程计
        if (imuEnable)
            topicPublishImu(); // 发布imu里程计
        if (ultraEnale)
        {
            topicPublishUltra(ULTRA_LF); // 发布左前超声波
            topicPublishUltra(ULTRA_MF); // 发布中前超声波
            topicPublishUltra(ULTRA_RF); // 发布右前超声波
            topicPublishUltra(ULTRA_MB); // 发布中后超声波
        }
        uart->transmitHart(); // 330ms
        // 机器人运动控制
        if (speedSetting.counter < 2)
        {
            speedSetting.counter++;
            uart->speedControl(speedSetting.speedLineX, speedSetting.speedLineY, speedSetting.speedAngle);
        }

        ros::spinOnce();   // 回调话题通信
        loop_rate.sleep(); // ROS线程周期控制
    }
    uart->close(); // 关闭串口
    return 0;
}

#pragma region 传感器数据话题发布

/**
 * @brief 超声波数据话题发布
 *
 * @param ultra 超声波编号
 */
void topicPublishUltra(UltraEnum ultra)
{
    sensor_msgs::Range msgUltra;                 // 定义超声波消息类型
    msgUltra.header.stamp = ros::Time::now();    // 超声波消息的时间戳
    msgUltra.header.frame_id = "base_footprint"; // 超声波消息的坐标系
    msgUltra.field_of_view = 0.25;               // 超声波消息的视野角度:rad（<15°）
    msgUltra.min_range = 0.03;                   // 超声波消息的最小距离:m
    msgUltra.max_range = 0.5;                    // 超声波消息的最大距离:m

    switch (ultra)
    {
    case UltraEnum::ULTRA_LF:                            // 左前
        if (uart->ultraStr.ultraLF < msgUltra.min_range) // 距离数据保护
            msgUltra.range = msgUltra.min_range;
        else if (uart->ultraStr.ultraLF > msgUltra.max_range)
            msgUltra.range = msgUltra.max_range;
        else
            msgUltra.range = uart->ultraStr.ultraLF;
        pubUltraLF.publish(msgUltra); // 发布超声波话题数据
        break;

    case UltraEnum::ULTRA_MF:                            // 中前
        if (uart->ultraStr.ultraMF < msgUltra.min_range) // 距离数据保护
            msgUltra.range = msgUltra.min_range;
        else if (uart->ultraStr.ultraMF > msgUltra.max_range)
            msgUltra.range = msgUltra.max_range;
        else
            msgUltra.range = uart->ultraStr.ultraMF;
        pubUltraMF.publish(msgUltra); // 发布超声波话题数据
        break;

    case UltraEnum::ULTRA_RF:                            // 右前
        if (uart->ultraStr.ultraRF < msgUltra.min_range) // 距离数据保护
            msgUltra.range = msgUltra.min_range;
        else if (uart->ultraStr.ultraRF > msgUltra.max_range)
            msgUltra.range = msgUltra.max_range;
        else
            msgUltra.range = uart->ultraStr.ultraRF;
        pubUltraRF.publish(msgUltra); // 发布超声波话题数据
        break;

    case UltraEnum::ULTRA_MB:                            // 中后
        if (uart->ultraStr.ultraMB < msgUltra.min_range) // 距离数据保护
            msgUltra.range = msgUltra.min_range;
        else if (uart->ultraStr.ultraMB > msgUltra.max_range)
            msgUltra.range = msgUltra.max_range;
        else
            msgUltra.range = uart->ultraStr.ultraMB;
        pubUltraMB.publish(msgUltra); // 发布超声波话题数据
        break;
    }
}

/**
 * @brief 里程计数据话题发布
 */
void topicPublishOdom(void)
{
    nav_msgs::Odometry msgOdom;                                        // 定义里程计（odom）消息类型
    geometry_msgs::Quaternion rotation;                                // 定义四元数变量
    rotation = tf::createQuaternionMsgFromYaw(uart->odomStr.angleRad); // odom的偏航角需要转换成四元数才能发布

    //[TF] odom坐标系转换
    if (!imuEnable) // 启动IMU融合里程计后不发布编码器odom坐标系
    {
        geometry_msgs::TransformStamped tfOdom;               // 定义坐标转换（tf）消息类型
        tfOdom.header.stamp = ros::Time::now();               // 载入tf时间戳
        tfOdom.header.frame_id = "odom";                      // 定义TF父子关系
        tfOdom.child_frame_id = "base_footprint";             // 定义TF父子关系
        tfOdom.transform.translation.x = uart->odomStr.moveX; // tf关于x轴的数据
        tfOdom.transform.translation.y = uart->odomStr.moveY; // tf关于y轴的数据
        tfOdom.transform.translation.z = 0.0;                 // tf关于z轴的数据
        tfOdom.transform.rotation = rotation;                 // tf航向参数
        static tf::TransformBroadcaster odomBroadcaster;      // 定义tf发布器
        odomBroadcaster.sendTransform(tfOdom);                // 发布从odom到base_footprint的坐标变换
    }

    //[Topic] odom话题数据
    msgOdom.header.stamp = ros::Time::now();                 // 载入odom时间戳
    msgOdom.header.frame_id = "odom";                        // 定义TF父子关系
    msgOdom.child_frame_id = "base_footprint";               // 定义TF父子关系
    msgOdom.pose.pose.position.x = uart->odomStr.moveX;      // odom关于x轴的数据
    msgOdom.pose.pose.position.y = uart->odomStr.moveY;      // odom关于x轴的数据
    msgOdom.pose.pose.position.z = 0.0;                      // odom关于z轴的数据
    msgOdom.pose.pose.orientation = rotation;                // odom航向角
    msgOdom.twist.twist.linear.x = uart->odomStr.speedLineX; // X轴线速度
    // msgOdom.twist.twist.linear.y = uart->odomStr.speedLineY;  // Y轴线速度
    msgOdom.twist.twist.angular.z = uart->odomStr.speedAngle; // odom角速度

    // 里程计协方差矩阵：
    // 如果机器人未运动，说明编码器的误差会比较小，认为编码器数据更可靠
    if (msgOdom.twist.twist.linear.x == 0 && msgOdom.twist.twist.linear.y == 0)
    {
        msgOdom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,
                                   0, 1e-3, 1e-9, 0, 0, 0,
                                   0, 0, 1e6, 0, 0, 0,
                                   0, 0, 0, 1e6, 0, 0,
                                   0, 0, 0, 0, 1e6, 0,
                                   0, 0, 0, 0, 0, 1e-9};

        msgOdom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,
                                    0, 1e-3, 1e-9, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e-9};
    }
    else // 考虑到运动中编码器可能带来的滑动误差，选择imu的数据更可靠
    {
        msgOdom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                                   0, 1e-3, 0, 0, 0, 0,
                                   0, 0, 1e6, 0, 0, 0,
                                   0, 0, 0, 1e6, 0, 0,
                                   0, 0, 0, 0, 1e6, 0,
                                   0, 0, 0, 0, 0, 1e3};
        msgOdom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3};
    }

    pubOdom.publish(msgOdom); // 发布odom话题数据
}

/**
 * @brief Imu里程计数据话题发布
 */
void topicPublishImu(void)
{
    sensor_msgs::Imu msgImu; // 定义（imu）消息类型

    msgImu.header.stamp = ros::Time::now(); // 载入imu时间戳
    msgImu.header.frame_id = "imu";         // imu坐标系名称

    // 线加速度
    msgImu.linear_acceleration.x = uart->imuStr.accx; // x轴线加速度
    msgImu.linear_acceleration.y = uart->imuStr.accy; // y轴线加速度
    msgImu.linear_acceleration.z = uart->imuStr.accz; // z轴线加速度
    // 角速度
    msgImu.angular_velocity.x = uart->imuStr.gyrox; // x轴角速度
    msgImu.angular_velocity.y = uart->imuStr.gyroy; // y轴角速度
    msgImu.angular_velocity.z = uart->imuStr.gyroz; // z轴角速度
    msgImu.angular_velocity_covariance[0] = 1e6;    // 三轴角速度协方差矩阵
    msgImu.angular_velocity_covariance[4] = 1e6;
    msgImu.angular_velocity_covariance[8] = 1e-6;
    // 四元素解算
    geometry_msgs::Quaternion orientation =
        tf::createQuaternionMsgFromYaw(uart->imuStr.yaw); // 只参考航向角解算四元素
    msgImu.orientation = orientation;
    msgImu.orientation_covariance[0] = 1e6;
    msgImu.orientation_covariance[4] = 1e6;
    msgImu.orientation_covariance[8] = 1e-6;

    pubImu.publish(msgImu); // 发布imu话题数据
}

/**
 * @brief 机器人速度话题发布
 *
 * @param cmd 键盘指令或遥控手柄指令
 */
void speedControl(const geometry_msgs::Twist &cmd) // 订阅/cmd_vel话题回调函数
{
    speedSetting.counter = 0;
    speedSetting.speedLineX = cmd.linear.x;
    speedSetting.speedLineY = cmd.linear.y;
    speedSetting.speedAngle = cmd.angular.z;
}

/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */
void exitSignal(int signum)
{
    uart->speedControl(0, 0, 0); // 机器人停止运动
    ROS_INFO_STREAM("------sebot_controller closeed!!!------");
    uart->close(); // 关闭串口
    exit(signum);
}

#pragma endregion