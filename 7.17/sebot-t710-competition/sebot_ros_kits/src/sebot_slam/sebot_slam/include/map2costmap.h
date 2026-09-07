/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
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

#ifndef __COSTMAP_CLIENT__
#define __COSTMAP_CLIENT__

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace autoSlam
{
  class Costmap2DClient
  {
  public:
    /**
     * @brief 构建客户端并开始倾听
     * @details 构造函数将阻塞，直到接收到第一个映射更新地图
     *
     * @param param_nh 需要检索的节点
     * @param subscription_nh 订阅主题的节点句柄
     * @param tf_listener 监听机器人的位姿变化
     */
    Costmap2DClient(ros::NodeHandle &param_nh, ros::NodeHandle &subscription_nh, const tf::TransformListener *tf_listener);
    /**
     * @brief 在代价地图的全局坐标系中获取机器人的姿态
     * @return 机器人在代价地图的全局坐标系中的位姿
     */
    geometry_msgs::Pose getRobotPose() const;

    /**
     * @brief 返回一个指向主代价地图映射的指针，它接收来自所有层的更新。
     *
     * This pointer will stay the same for the lifetime of Costmap2DClient object.
     */
    costmap_2d::Costmap2D *getCostmap()
    {
      return &costmap;
    }

    /**
     * @brief 返回一个指向主代价地图映射的指针，它接收来自所有层的更新。
     *
     * This pointer will stay the same for the lifetime of Costmap2DClient object.
     */
    const costmap_2d::Costmap2D *getCostmap() const
    {
      return &costmap;
    }

    /**
     * @brief  获取全局坐标系
     * @return 全局坐标系名称
     */
    const std::string &getGlobalFrameID() const
    {
      return global_frame;
    }

    /**
     * @brief  获取机器人基坐标系
     * @return 机器人基坐标系的名称
     */
    const std::string &getBaseFrameID() const
    {
      return robot_base_frame;
    }

  protected:
    void updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void updatePartialMap(const map_msgs::OccupancyGridUpdate::ConstPtr &msg);

    costmap_2d::Costmap2D costmap;

    const tf::TransformListener *const tf_; // 监听tf坐标变换
    std::string global_frame;               // 全局坐标系
    std::string robot_base_frame;           // 机器人基坐标系
    double transformTolerance;              // tf坐标转换超时忍耐时间
    std::string map_topic;                  // 地图话题名称

  private:
    ros::Subscriber costmap_sub; // 接收代价地图话题
  };

}

#endif
