/**
 * @file auto_slam.cpp
 * @author HC
 * @brief 自主建图主程序
 * @version 0.1
 * @date 2024/02/20 10:05:25
 * @copyright  :Copyright (c) 2023
 * @note 具体功能模块:
                  [1] 搜索地图数据上的边界点
                  [2] 选取最小代价的边界作为目标发布导航点
                  [3] 搜索完所有边界后结束自主建图并保存地图
 */

#include <auto_slam.h>
#include <cstdlib>
#include "std_msgs/String.h"

namespace autoSlam
{
  AutoSlam::AutoSlam()
      : privateNode("~"),                                      // 使用私有命名空间(仅使用以节点名称为前缀的参数)
        listenerTF(ros::Duration(10.0)),                       // 储存十秒内的坐标变换
        costmapClient(privateNode, relativeNode, &listenerTF), //
        move_base_client("move_base"), prevDistance(0), lastMarkersCount(0)
  {
    privateNode.param("planner_frequency", planFrequency, 0.33); // 计算新边界和重新考虑目标的频率
    privateNode.param("progress_timeout", timeout, 30.0);        // 容忍机器人没有任何进展的时间
    progressTimeout = ros::Duration(timeout);
    privateNode.param("visualize", visualize, false);             // 指定是否发布可视化边界
    privateNode.param("potential_scale", potentialScale, 1e-3);   // 边界距离的加权
    privateNode.param("gain_scale", gainScale, 1.0);              // 边界大小的加权
    privateNode.param("min_frontier_size", minFrontierSize, 0.5); // 设置最小边界
    privateNode.param("debug", debug, false);                     // 是否打印debug信息
    privateNode.param("audio", audio, false);                     // 是否语音播报机器人当前信息

    // 初始化边界搜索算法
    frontierSearch = frontier_exploration::FrontierSearch(costmapClient.getCostmap(), potentialScale, gainScale, minFrontierSize);

    if (visualize)
    {
      pubMarkerArray = privateNode.advertise<visualization_msgs::MarkerArray>("frontiers", 10); // 定义边界可视化话题
    }

    // 是否语音播报
    if (audio)
      pubAudio = privateNode.advertise<std_msgs::String>("/audio", 2); // 定义语音播报话题

    ROS_INFO("Waiting to connect to move_base server");
    move_base_client.waitForServer(); // 等待movebase服务启动
    ROS_INFO("Connected to move_base server");

    if (audio)
    {
      std_msgs::String msg;
      msg.data = "startSlam"; // 音频名称
      pubAudio.publish(msg);  // 发布需要播放的音频名称
    }
    // 等待超时容忍时间发布新的计划
    autoSlamTime = relativeNode.createTimer(ros::Duration(1. / planFrequency), [this](const ros::TimerEvent &)
                                            { makePlan(); });
  }

  AutoSlam::~AutoSlam() // 定义构析函数
  {
    if (!finish)
      stop(); // 程序退出时, 关闭自主建图程序
    finish = true;
  }

  /**
   * @brief 发布边界可视化
   */
  void AutoSlam::visualizeFrontiers(const std::vector<frontier_exploration::Frontier> &frontiers)
  {
    std_msgs::ColorRGBA blue; // 定义红绿蓝三种颜色的RGB值
    std_msgs::ColorRGBA red;
    std_msgs::ColorRGBA green;
    blue.r = 0;
    blue.g = 0;
    blue.b = 1.0;
    blue.a = 1.0;
    red.r = 1.0;
    red.g = 0;
    red.b = 0;
    red.a = 1.0;
    green.r = 0;
    green.g = 1.0;
    green.b = 0;
    green.a = 1.0;

    ROS_DEBUG("visualising %lu frontiers", frontiers.size());
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker> &markers = markers_msg.markers;
    visualization_msgs::Marker m;

    m.header.frame_id = costmapClient.getGlobalFrameID(); // 获取全局坐标系
    m.header.stamp = ros::Time::now();
    m.ns = "frontiers";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
    // lives forever
    m.lifetime = ros::Duration(0);
    m.frame_locked = true;

    // 使用三元条件运算符进行赋值, 如果不为空则赋值为容器中第一个元素
    double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

    m.action = visualization_msgs::Marker::ADD; // 更改标记点的动作为添加
    size_t id = 0;
    for (auto &frontier : frontiers)
    {
      m.type = visualization_msgs::Marker::POINTS; // 标记点的类型为点，绘制出边界线
      m.id = int(id);
      m.pose.position = {};
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      if (goalOnBlacklist(frontier.centroid)) // 如果边界质心在黑名单中则更改边界标记颜色为红色, 否则为蓝色
      {
        m.color = red;
      }
      else
      {
        m.color = blue;
      }
      markers.push_back(m); // 发布标记
      ++id;
      m.type = visualization_msgs::Marker::SPHERE; // 标记点的类型为球形，绘制出代价边界点
      m.id = int(id);
      m.pose.position = frontier.initial;

      // 根据代价对边界进行缩放(代价越高的边界越小)
      double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      m.points = {};
      m.color = green;
      markers.push_back(m);
      ++id;
    }
    size_t current_markers_count = markers.size(); // 获取标记点数量

    // 删除不再使用的标记点
    m.action = visualization_msgs::Marker::DELETE; // 更改标记点的动作为删除
    for (; id < lastMarkersCount; ++id)
    { // 根据上一次储存的标记点位置，删除上一次所有标记点
      m.id = int(id);
      markers.push_back(m);
    }

    lastMarkersCount = current_markers_count; // 更新标记点计数
    pubMarkerArray.publish(markers_msg);      // 发布所有标记点的状态，如添加和删除
  }
  /**
   * @brief 发布目标点
   */
  void AutoSlam::makePlan()
  {
    auto pose = costmapClient.getRobotPose();                  // 获取机器人位姿
    auto frontiers = frontierSearch.searchFrom(pose.position); // 根据机器人位置进行边界搜索
    ROS_DEBUG("found %lu frontiers", frontiers.size());        // 打印边界数量
    for (size_t i = 0; i < frontiers.size(); ++i)
    {
      ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost); // 打印所有边界信息
    }

    // 是否发布为可视化边界
    if (visualize && !finish)
    {
      visualizeFrontiers(frontiers);
    }

    if (!getInitial)
    {
      initial = pose.position; // 获取机器人初始位置
      getInitial = true;
    }

    if (frontiers.empty()|| finish)
    { // 如果边界为空则自主建图完成
      if (!finish)
      {
        backHome(); // 机器人回到原位
      }
      return;
    }
    // 寻找第一个未被列入黑名单的边界(代价最小的边界)
    // find_if_not 寻找第一个不满足条件的元素
    auto frontier =
        std::find_if_not(frontiers.begin(), frontiers.end(),
                         [this](const frontier_exploration::Frontier &f)
                         {
                           return goalOnBlacklist(f.centroid);
                         });

    if (frontier == frontiers.end() || finish) // 如果该边界是最后一个边界则自主建图完成
    {
      if (!finish)
      {
        backHome(); // 机器人回到原位
      }
      return;
    }
    geometry_msgs::Point targetPosition = frontier->centroid; // 将边界质心为目标点

    bool same_goal = prevGoal == targetPosition; // 防止目标点重复
    prevGoal = targetPosition;                   // 记录目标点

    // 如果目标不同或者当前最小距离小于上一次记录的最小距离则认为当前有进展(更换目标点或者接近目标点)
    if (!same_goal || prevDistance > frontier->minDistance)
    {
      lastProgress = ros::Time::now();      // 记录最后有进展的时间
      prevDistance = frontier->minDistance; // 更新记录的最小距离
    }
    // 如果长时间未进展则会将该目标点拉近黑名单 progressTimeout 参数设定的超时时间
    if (ros::Time::now() - lastProgress > progressTimeout)
    {
      frontierBlacklist.push_back(targetPosition); // 加入黑名单
      ROS_DEBUG("Adding current goal to black list");
      makePlan(); // 重新计划目标点
      return;
    }

    if (same_goal) // 如果目标相同则不做任何操作
      return;
    
    // 改变目标后将新的目标点发送到move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position = targetPosition;
    goal.target_pose.pose.orientation.w = 1.;                            // 表示没有旋转
    goal.target_pose.header.frame_id = costmapClient.getGlobalFrameID(); // 全局坐标系
    goal.target_pose.header.stamp = ros::Time::now();                    // 记录时间戳
    move_base_client.sendGoal(goal, [this, targetPosition](
                                        const actionlib::SimpleClientGoalState &status,
                                        const move_base_msgs::MoveBaseResultConstPtr &result)
                              {
                                reachedGoal(status, result, targetPosition); // 已到达目标点
                              });
    ROS_INFO("[---] publish goal !!!");
  }

  /**
   * @brief 检查目标点是否接近黑名单
   * @param goal 需要判断的目标点
   */
  bool AutoSlam::goalOnBlacklist(const geometry_msgs::Point &goal)
  {
    constexpr static size_t tolerace = 5;                          // 设置目标点与黑名单中点的容差范围
    costmap_2d::Costmap2D *costmap2d = costmapClient.getCostmap(); // 获取代价地图数据

    // 检查目标点是否在黑名单上
    for (auto &frontier_goal : frontierBlacklist)
    {
      double x_diff = fabs(goal.x - frontier_goal.x); // 计算黑名单和目标点的差异
      double y_diff = fabs(goal.y - frontier_goal.y);

      if (x_diff < tolerace * costmap2d->getResolution() && // 如果差异较小则可认为该目标点接近黑名单
          y_diff < tolerace * costmap2d->getResolution())
        return true;
    }
    return false;
  }

  /**
   * @brief 机器人回到原点
   * @param
   */
  void AutoSlam::backHome()
  {
    if (audio)
    {
      std_msgs::String msg;
      msg.data = "finishSlam"; // 音频名称
      pubAudio.publish(msg);   // 发布需要播放的音频名称
    }
    ROS_INFO("auto_slam has been completed!");
    ROS_INFO("Now prepare to return to the starting position!");
    finish = true;
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position = initial;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(6.283); // 机器人的朝向 1.57 弧度 == 90° x轴正方向为0° y轴正方向为90°
    goal.target_pose.header.frame_id = costmapClient.getGlobalFrameID();       // 全局坐标系
    goal.target_pose.header.stamp = ros::Time::now();                          // 记录时间戳
    move_base_client.sendGoal(goal, [this](
                                        const actionlib::SimpleClientGoalState &status,
                                        const move_base_msgs::MoveBaseResultConstPtr &result)
                              {
                                reachedGoal(status, result, this->initial); // 已到达目标点
                              });
  }

  /**
   * @brief 到达目标点
   * @param status 机器人导航状态
   * @param frontier_goal 导航点坐标
   */
  void AutoSlam::reachedGoal(const actionlib::SimpleClientGoalState &status,
                             const move_base_msgs::MoveBaseResultConstPtr &,
                             const geometry_msgs::Point &frontier_goal)
  {
    ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
    if (status == actionlib::SimpleClientGoalState::ABORTED) // 如果目标执行被中止
    {
      frontierBlacklist.push_back(frontier_goal); // 将目标点添加进黑名单防止重复搜索
      ROS_DEBUG("Adding current goal to black list");
    }

    if (!finish)
    {
      // 无视任何频率条件立即寻找新目标
      oneshot = relativeNode.createTimer(
          ros::Duration(0, 0), [this](const ros::TimerEvent &)
          { makePlan(); },
          true);
    }

    else 
    {
      stop();
    }
  }

  /**
   * @brief 开始自主建图
   */
  void AutoSlam::start()
  {
    autoSlamTime.start();
    startTime = std::chrono::high_resolution_clock::now(); // 开始计时
  }

  /**
   * @brief 停止自主建图
   */
  void AutoSlam::stop()
  {
    if (audio)
    {
      std_msgs::String msg;
      msg.data = "backHome"; // 音频名称
      pubAudio.publish(msg); // 发布需要播放的音频名称
    }
    endTime = std::chrono::high_resolution_clock::now();                                        // 获取结束时间
    move_base_client.cancelAllGoals();                                                          // 取消movebase的所有目标
    autoSlamTime.stop();                                                                        // 关闭自主建图
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime); // 计算执行时间（以毫秒为单位）
    // 保存地图的指令
    if (finish)
    {
      system("gnome-terminal -- roslaunch sebot_slam sebot_map_save.launch");
    }
    ROS_INFO("Exploration stopped.");
    ROS_INFO("Execution time: %ld s", duration.count() / 1000);
  }

} // namespace autoSlam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_slam"); // 初始化自主建图节点
  sleep(5);                           // 等待其它节点正常启动
  autoSlam::AutoSlam autoSlam;        // 实例化自主建图类
  if (autoSlam.debug)                 // 只有在debug模式下才会打印debug日志信息
  {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged(); // 如果成功调整日志类型为Debug则及时更新日志
    }
  }

  ros::spin();

  return 0;
}