/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef __AUTO_SLAM__
#define __AUTO_SLAM__

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/MarkerArray.h>

#include <map2costmap.h>
#include <frontier_search.h>

namespace autoSlam
{
  /**
   * @class AutoSlam
   * @brief 依附于robot_actions::Action接口，它发布导航点来控制机器人完成自主建图。
   */
  class AutoSlam
  {
  public:
    AutoSlam();
    ~AutoSlam();

    void start();
    void stop();
    bool debug;                 // 是否打印debug信息
    bool audio;                 // 是否语音播报
    std::string audioNum = "1"; // 需要播放的音频编号

  private:
    void makePlan(); // 指定全局路径规划

    void visualizeFrontiers(const std::vector<frontier_exploration::Frontier> &frontiers); // 发布边界作为标记

    // 当机器人到达目标时调用的回调函数。
    void reachedGoal(const actionlib::SimpleClientGoalState &status, const move_base_msgs::MoveBaseResultConstPtr &result, const geometry_msgs::Point &frontier_goal);

    bool goalOnBlacklist(const geometry_msgs::Point &goal); // 检查目标点是否在黑名单上
    void backHome();                                        // 机器人回到原位
    ros::NodeHandle privateNode;
    ros::NodeHandle relativeNode;
    ros::Publisher pubMarkerArray;    // 定义可视化边界发布者
    ros::Publisher pubAudio;          // 定义语音播报发布者
    tf::TransformListener listenerTF; // 用于监听和转换坐标系的tf

    Costmap2DClient costmapClient; // 代价地图转化
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
    frontier_exploration::FrontierSearch frontierSearch; // 边界搜索
    ros::Timer autoSlamTime;                             // ROS定时器，用于记录探索时间
    ros::Timer oneshot;                                  // 用于执行单次探索

    std::vector<geometry_msgs::Point> frontierBlacklist; // 创建容器用于储存边界探索黑名单
    geometry_msgs::Point prevGoal;                       // 用于储存前一个目标的坐标信息
    double prevDistance;                                 // 用于储存前一个目标的距离
    ros::Time lastProgress;                              // 最后一次进步的时间
    size_t lastMarkersCount;                             // 用于储存上一组标记点的数量

    // 算法参数
    double minFrontierSize;                                                                               // 将边界视为探索目标的边界的最小大小
    double planFrequency;                                                                                 // 计算新边界和重新考虑目标的频率
    double potentialScale, gainScale;                                                                     // 到边界的距离和边界大小用于计算加权边界
    double timeout;                                                                                       // 容忍超时的时间
    ros::Duration progressTimeout;                                                                        // 等待一个容忍超时的时间
    bool visualize;                                                                                       // 是否发布可视化边界
    bool finish = false;                                                                                  // 自主建图已完成
    bool getInitial = false;                                                                              // 获取初始点
    geometry_msgs::Point initial;                                                                         // 初始点坐标
    std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now(); // 程序开始时间
    std::chrono::high_resolution_clock::time_point endTime = std::chrono::high_resolution_clock::now();   // 程序结束时间
  };
}

#endif
