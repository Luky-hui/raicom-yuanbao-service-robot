/**
 * @file robotTransform.cpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 机器人坐标系变换（静态坐标系发布）
 * @version 0.1
 * @date 2023-10-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sebot_transform");
    tf::TransformBroadcaster tfBroadcaster; // 坐标系数据发布器

    //[01] 定义IMU静态坐标系
    tf::Transform tfImu;
    tf::Quaternion quater;                 // 四元数变换
    quater.setRPY(0, 0, 0);                // 空间向量 rad q.setRPY(roll, pitch, yaw);
    tfImu.setOrigin(tf::Vector3(0, 0, 0)); // 三维坐标
    tfImu.setRotation(quater);

    //[02] 定义激光雷达静态坐标系
    tf::Transform tfLaser;
    tfLaser.setOrigin(tf::Vector3(0, 0, 0.15)); // 三维坐标: m
    quater.setRPY(0, 0, -3.1416);               // 空间向量: rad q.setRPY(roll, pitch, yaw);
    tfLaser.setRotation(quater);

    //[03] 定义深度相机静态坐标系
    tf::Transform tfCamera;
    tfCamera.setOrigin(tf::Vector3(0.05, 0, 1)); // 三维坐标
    quater.setRPY(0, 0, 0);                      // 空间向量 q.setRPY(roll, pitch, yaw);
    tfCamera.setRotation(quater);

    //[04] 定义超声波静态坐标系
    tf::Transform tfUltraLF;
    tfUltraLF.setOrigin(tf::Vector3(0.2, 0.1, 0.07)); // 三维坐标
    quater.setRPY(0, 0, 0);                           // 空间向量 q.setRPY(roll, pitch, yaw);
    tfUltraLF.setRotation(quater);

    //[05] 定义超声波静态坐标系
    tf::Transform tfUltraMF;
    tfUltraMF.setOrigin(tf::Vector3(0.2, 0, 0.07)); // 三维坐标
    quater.setRPY(0, 0, 0);                         // 空间向量 q.setRPY(roll, pitch, yaw);
    tfUltraMF.setRotation(quater);

    //[06] 定义超声波静态坐标系
    tf::Transform tfUltraRF;
    tfUltraRF.setOrigin(tf::Vector3(0.2, -0.1, 0.07)); // 三维坐标
    quater.setRPY(0, 0, 0);                            // 空间向量 q.setRPY(roll, pitch, yaw);
    tfUltraRF.setRotation(quater);

    //[07] 定义超声波静态坐标系
    tf::Transform tfUltraMB;
    tfUltraMB.setOrigin(tf::Vector3(-0.2, 0, 0.07)); // 三维坐标
    quater.setRPY(0, 0, -3.1416);                    // 空间向量 q.setRPY(roll, pitch, yaw);
    tfUltraMB.setRotation(quater);

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        tfBroadcaster.sendTransform(tf::StampedTransform(tfImu, ros::Time::now(), "base_footprint", "imu"));
        tfBroadcaster.sendTransform(tf::StampedTransform(tfLaser, ros::Time::now(), "base_footprint", "laser"));
        tfBroadcaster.sendTransform(tf::StampedTransform(tfCamera, ros::Time::now(), "base_footprint", "camera"));
        tfBroadcaster.sendTransform(tf::StampedTransform(tfUltraLF, ros::Time::now(), "base_footprint", "ultraLF"));
        tfBroadcaster.sendTransform(tf::StampedTransform(tfUltraMF, ros::Time::now(), "base_footprint", "ultraMF"));
        tfBroadcaster.sendTransform(tf::StampedTransform(tfUltraRF, ros::Time::now(), "base_footprint", "ultraRF"));
        tfBroadcaster.sendTransform(tf::StampedTransform(tfUltraMB, ros::Time::now(), "base_footprint", "ultraMB"));
        loop_rate.sleep();
    }

    return 0;
};
