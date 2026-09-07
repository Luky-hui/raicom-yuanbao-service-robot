#include <map2costmap.h>
#include <functional>
#include <mutex>
#include <string>

namespace autoSlam
{
  // 静态翻译表可以加快地图转化的速度
  std::array<unsigned char, 256> init_translation_table();
  static const std::array<unsigned char, 256> cost_translation_table = init_translation_table();

  /**
   * @brief 代价地图客户端
   * @details 确保所需的参数和坐标变换都已正确设置和可用，并订阅地图话题以更新地图数据
   */
  Costmap2DClient::Costmap2DClient(ros::NodeHandle &param_nh, ros::NodeHandle &subscription_nh, const tf::TransformListener *tf) : tf_(tf)
  {
    param_nh.param("map_topic", map_topic, std::string("slam_map"));                     // 地图话题名称
    param_nh.param("robot_base_frame", robot_base_frame, std::string("base_footprint")); // 机器人底盘坐标系
    param_nh.param("global_frame", global_frame, std::string("map"));                    // 机器人参考坐标系 如map、odom
    param_nh.param("transform_tolerance", transformTolerance, 0.3);                      // 机器人变换容差

    // 订阅地图话题调用updateFullMap函数更新地图
    costmap_sub = subscription_nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1000,
                                                                     [this](const nav_msgs::OccupancyGrid::ConstPtr &msg)
                                                                     { updateFullMap(msg); });
    ROS_INFO("Waiting for map to become available, topic: %s", map_topic.c_str());
    auto costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, subscription_nh); // 等待地图数据更新，验证地图话题可用
    updateFullMap(costmap_msg);                                                                         // 立即更新一次地图

    // 我们需要确保机器人地盘坐标系和全局坐标系可用
    /* tf变换对于getRobotPose是必需的 */
    ros::Time last_error = ros::Time::now(); // 记录上一次错误信息的时间
    std::string tf_error;
    while (ros::ok() &&
           !tf_->waitForTransform(global_frame, robot_base_frame, ros::Time(), // 在成功接收到坐标变换之前将一直循环
                                  ros::Duration(0.1), ros::Duration(0.01),
                                  &tf_error))
    {
      ros::spinOnce();
      if (last_error + ros::Duration(5.0) < ros::Time::now()) // 检查是否已经过去了5秒而没有成功获取变换，仍没有获取则会每五秒打印一次错误信息
      {
        ROS_WARN(
            "Timed out waiting for transform from %s to %s to become available "
            "before subscribing to map, tf error: %s",
            robot_base_frame.c_str(), global_frame.c_str(), tf_error.c_str());
        last_error = ros::Time::now();
      }

      tf_error.clear();
    }
  }

  /**
   * @brief 更新并转化地图
   * @details 将普通的地图转化为代价地图
   */
  void Costmap2DClient::updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    // 提取地图参数
    unsigned int sizeX = msg->info.width; // 地图的尺寸
    unsigned int sizeY = msg->info.height;
    double resolution = msg->info.resolution;     // 地图的分辨率
    double originX = msg->info.origin.position.x; // 地图初始位置
    double originY = msg->info.origin.position.y;
    int radius = ((0.5/resolution)+1)/2;       // 机器人所在栅格数量,机器人膨胀半径:0.5
    std::vector<size_t> deadly;

    // 调整地图大小
    ROS_DEBUG("received full new map, resizing to: %d, %d", sizeX, sizeY);
    costmap.resizeMap(sizeX, sizeY, resolution, originX, originY); // 根据当前地图大小来更新新地图信息

    // 加入互斥锁保护地图数据
    auto *mutex = costmap.getMutex();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

    // 用数据填充地图
    unsigned char *costmapData = costmap.getCharMap();                          // 获取代价地图的原始数据指针
    size_t costmapSize = costmap.getSizeInCellsX() * costmap.getSizeInCellsY(); // 计算代价地图的总大小
    ROS_DEBUG("full map update, %lu values", costmapSize);                      // 表明正在更新完整的地图，并显示地图的总单元格数。
    for (size_t i = 0; i < costmapSize && i < msg->data.size(); ++i)
    {
      unsigned char cellCost = static_cast<unsigned char>(msg->data[i]); // 将地图数据转化为拥有代价的数据
      if (cellCost == 100)  // 将致命区域栅格编号添加进容器
      {
        deadly.push_back(i);
      }
      costmapData[i] = cost_translation_table[cellCost];
    }

    for (const auto& num : deadly) // 生成代价地图层  假定radius = 8
    {                                                 
      if (num % sizeX > radius) // 检查该索引是否为当前行的前8个单元格
        costmapData[num-radius] = 254;

      if (num % sizeX < sizeX - radius) // 检查该索引是否为当前行的最后8个单元格
        costmapData[num+radius] = 254;

      if (num >= sizeX * radius)     // 检查该索引是否为第8行以上的单元格
        costmapData[num - sizeX * radius] = 254; 

      if (num < sizeX * (sizeY - radius)) // 检查该索引是否为最下面8行的单元格
        costmapData[num + sizeX * radius] = 254;


      if (num % sizeX > radius && num >= sizeX * radius)  // 检查该索引左上角是否可以有单元格
        costmapData[num - radius - sizeX * radius] = 254;

      if (num % sizeX > radius && num < sizeX * (sizeY - radius)) // 检查该索引左下角是否可以有单元格
        costmapData[num - radius + sizeX * radius] = 254;

      if (num % sizeX < sizeX - radius && num >= sizeX * radius)  // 检查该索引右上角是否可以有单元格
        costmapData[num + radius - sizeX] = 254;

      if (num % sizeX < sizeX - radius && num < sizeX * (sizeY - radius)) // 检查该索引右下角是否可以有单元格
        costmapData[num + radius + sizeX] = 254;
    }

    ROS_DEBUG("map updated, written %lu values", costmapSize); // 更新地图提示
  }

  /**
   * @brief 获取机器人姿态
   * @details 获取机器人当前位置和姿态，给后续搜索算法提供初始位置
   */
  geometry_msgs::Pose Costmap2DClient::getRobotPose() const
  {
    tf::Stamped<tf::Pose> global_pose; // 储存机器人的全局姿态
    tf::Stamped<tf::Pose> robot_pose;  // 储存机器人在本地坐标系中的姿态
    global_pose.setIdentity();         // 将其设置为单位位姿初始化
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // 记录时间，在之后检查tf延迟

    // 获取机器人的全局姿态
    try
    {
      tf_->transformPose(global_frame, robot_pose, global_pose);
    }
    catch (tf::LookupException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot "
                              "pose: %s\n",
                         ex.what());
      return {};
    }
    catch (tf::ConnectivityException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n",
                         ex.what());
      return {};
    }
    catch (tf::ExtrapolationException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n",
                         ex.what());
      return {};
    }

    // 检查全局姿态超时，transform_tolerance_为机器人的变换容差
    if (current_time.toSec() - global_pose.stamp_.toSec() >
        transformTolerance)
    {
      ROS_WARN_THROTTLE(1.0, "Costmap2DClient transform timeout. Current time: "
                             "%.4f, global_pose stamp: %.4f, tolerance: %.4f",
                        current_time.toSec(), global_pose.stamp_.toSec(),
                        transformTolerance);
      return {};
    }

    geometry_msgs::PoseStamped msg;
    tf::poseStampedTFToMsg(global_pose, msg); // 转换消息类型

    return msg.pose;
  }

  /**
   * @brief 地图数据转化表
   * @details 定义一个代价翻译表，用于将地图转化为代价
   */
  std::array<unsigned char, 256> init_translation_table()
  {
    std::array<unsigned char, 256> cost_translation_table;

    // lineary mapped from [0..100] to [0..255]
    for (size_t i = 0; i < 256; ++i)
    {
      cost_translation_table[i] = static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
    }

    // special values:
    cost_translation_table[0] = 0;                                // 没有障碍
    cost_translation_table[99] = 253;                             // 内切障碍
    cost_translation_table[100] = 254;                            // 致命障碍
    cost_translation_table[static_cast<unsigned char>(-1)] = 255; // 未知区域

    return cost_translation_table;
  }

}
