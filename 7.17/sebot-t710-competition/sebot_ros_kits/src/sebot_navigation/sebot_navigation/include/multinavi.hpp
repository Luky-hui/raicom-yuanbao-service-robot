#pragma once
/**
 * @file multinavi.hpp
 * @author Leo ()
 * @brief ROS机器人多点导航
 * @version 0.1
 * @date 2024-02-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h> // 处理动作客户端
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h> // 带有时间戳的三位位置和方向
#include <ros/ros.h>
#include "ros/time.h"
#include <fstream>
#include <iostream>
#include <string>
#include "tinyxml.h"
#include <tf/transform_datatypes.h>
#include <unistd.h> // 包含sleep函数的头文件

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> rosAction;

class MultiNavi
{
private:
  ros::NodeHandle rosNode;
  ros::Timer timer_; // 用于声明回调函数
  move_base_msgs::MoveBaseGoal targetPose;
  rosAction moveBaseAction;
  std::vector<move_base_msgs::MoveBaseGoal> gaolsMulti; // 多点导航目标点
  bool initialPoint = false;                            // 初始点
  ros::Duration waitTime;                               // 设定导航等待时间
  ros::Time timer;                                      // 计时器，用于记录 导航后的等待时间
  int index = 1;                                        // 导航点下标
  bool task;                                            // 机器人当前是否在执行任务
  geometry_msgs::Point initial;                         // 记录机器人初始坐标
  void callbackTimer(const ros::TimerEvent &event);
  void activeCb();
  void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
  void readXml(const std::string &xmlPath);                    // 读取XML文件
  std::vector<double> readDouble(const std::string &data);     // 转换double类型
  std::string findPackagePath(const std::string &packageName); // 获取功能包路径
public:
  ros::Publisher amcl_pose_pub = rosNode.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10); // 创建发布者，发布到/amcl_pose话题，队列大小为10
  MultiNavi();
  ~MultiNavi();
};
