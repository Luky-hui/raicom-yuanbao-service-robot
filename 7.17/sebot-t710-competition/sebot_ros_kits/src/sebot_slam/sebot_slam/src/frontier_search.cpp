#include <frontier_search.h>
#include <mutex>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <bfs.h>

namespace frontier_exploration
{
  using costmap_2d::FREE_SPACE;
  using costmap_2d::LETHAL_OBSTACLE;
  using costmap_2d::NO_INFORMATION;

  /**
   * @brief 初始化FrontierSearch对象
   * @param costmap 地图数据
   * @param potentialScale 边界距离加权
   * @param gainScale 边界大小加权
   * @param minFrontierSize 最小边界大小
   */
  FrontierSearch::FrontierSearch(costmap_2d::Costmap2D *costmap,
                                 double potentialScale, double gainScale,
                                 double minFrontierSize)
      : costmap(costmap) // 地图数据
        ,
        potentialScale(potentialScale) // 边界距离加权
        ,
        gainScale(gainScale) // 边界大小加权
        ,
        minFrontierSize(minFrontierSize) // 最小边界大小
  {
  }

  /**
   * @brief
   * @param position 机器人的位姿
   */
  std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
  {
    // 初始化边界容器
    std::vector<Frontier> frontierList;

    // 在搜索之前检查机器人的位置是否位于地图外侧
    unsigned int mx, my;
    if (!costmap->worldToMap(position.x, position.y, mx, my))
    {
      ROS_ERROR("Robot out of map bounds, cannot search for frontiers");
      return frontierList;
    }

    // 确保地图一致性，并且在搜索期间锁定地图刷新
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    map = costmap->getCharMap();        // 获取地图代价
    sizeX = costmap->getSizeInCellsX(); // 获取地图大小
    sizeY = costmap->getSizeInCellsY();

    // 初始化标志数组用来跟踪访问的单元格和边界单元格
    std::vector<bool> frontierFlag(sizeX * sizeY, false);
    std::vector<bool> visitedFlag(sizeX * sizeY, false);

    // 初始化广度优先搜索
    std::queue<unsigned int> bfs;

    // 找到最近的空白单元格开始搜索
    unsigned int clear, pos = costmap->getIndex(mx, my); // 获取(mx, my)在代价地图中的索引
    // 尝试在机器人周围寻找空白单元格，如果找不到则使用机器人所在单元格
    if (nearestCell(clear, pos, FREE_SPACE, *costmap))
    {
      bfs.push(clear);
    }
    else
    {
      bfs.push(pos);
      ROS_WARN("Could not find nearby clear cell to start search");
    }
    visitedFlag[bfs.front()] = true; // 防止重复搜索，将该点添加进已访问列表

    while (!bfs.empty()) // 如果不为空
    {
      unsigned int idx = bfs.front(); // 获取队列中第一个单元格索引
      bfs.pop();                      // 移除第一个单元格

      // 遍历四连通区域将所有空闲的、未访问的单元格添加到队列中降序搜索
      for (unsigned nbr : nhood4(idx, *costmap))
      {
        if (map[nbr] <= map[idx] && !visitedFlag[nbr]) // 当前单元格的代价小于等于初始单元格并且未访问过
        {
          visitedFlag[nbr] = true;
          bfs.push(nbr);
        }
        else if (isNewFrontierCell(nbr, frontierFlag)) // 判断是否可以发展为边界
        {
          frontierFlag[nbr] = true;
          Frontier frontierNew = buildNewFrontier(nbr, pos, frontierFlag);    // 创建新的边界
          if (frontierNew.size * costmap->getResolution() >= minFrontierSize) // 新的边界满足最小边界要求
          {
            frontierList.push_back(frontierNew); // 添加进边界列表
          }
        }
      }
    }

    // 设置边界代价
    for (auto &frontier : frontierList) // 遍历边界列表
    {
      frontier.cost = frontierCost(frontier); // 计算边界代价
    }
    // 根据代价值对边界列表进行排序(从小到大)
    std::sort(
        frontierList.begin(), frontierList.end(),
        [](const Frontier &f1, const Frontier &f2)
        { return f1.cost < f2.cost; });

    return frontierList;
  }

  /**
   * @brief 创建新的边界
   * @details 根据空白单元格来创建边界，并且计算到机器人的距离信息
   * @param initialCell 初始单元格
   * @param reference 参考单元格
   * @param frontierFlag 已知边界单元格列表
   */
  Frontier FrontierSearch::buildNewFrontier(unsigned int initialCell,
                                            unsigned int reference,
                                            std::vector<bool> &frontierFlag)
  {
    // 初始化边界结构
    Frontier output;
    output.centroid.x = 0; // 质心位置
    output.centroid.y = 0;
    output.size = 1;                                              // 大小
    output.minDistance = std::numeric_limits<double>::infinity(); // 初始边界距离无限大

    unsigned int ix, iy;
    costmap->indexToCells(initialCell, ix, iy);                      // 将单元格转化为索引
    costmap->mapToWorld(ix, iy, output.initial.x, output.initial.y); // 转换成现实世界中的坐标

    // 初始化广度优先算法并将初始网格单元推入队列
    std::queue<unsigned int> bfs;
    bfs.push(initialCell);

    unsigned int rx, ry;
    double referenceX, referenceY;
    costmap->indexToCells(reference, rx, ry);            // 将单元格转化为索引
    costmap->mapToWorld(rx, ry, referenceX, referenceY); // 转换成现实世界中的坐标

    while (!bfs.empty())
    {
      unsigned int idx = bfs.front();
      bfs.pop();

      // 尝试将8连通单元格添加到边界队列
      for (unsigned int nbr : nhood8(idx, *costmap))
      {
        // 检查相邻单元格是否是潜在的前沿单元格
        if (isNewFrontierCell(nbr, frontierFlag))
        {
          // 将单元格标记为边界
          frontierFlag[nbr] = true;
          unsigned int mx, my;
          double wx, wy;
          costmap->indexToCells(nbr, mx, my);  // 将单元格转化为索引
          costmap->mapToWorld(mx, my, wx, wy); // 转换成现实世界中的坐标

          // 创建一个ROS geometry_msgs::Point消息将该单元格储存为导航点
          geometry_msgs::Point point;
          point.x = wx;
          point.y = wy;
          output.points.push_back(point);

          // 更新边界大小
          output.size++;

          // 更新边界的质心位置
          output.centroid.x += wx;
          output.centroid.y += wy;

          // 计算前沿单元格到机器人位置的距离
          double distance = sqrt(pow((double(referenceX) - double(wx)), 2.0) +
                                 pow((double(referenceY) - double(wy)), 2.0));
          if (distance < output.minDistance)
          {
            output.minDistance = distance;
            output.middle.x = wx;
            output.middle.y = wy;
          }

          // 添加到搜索队列
          bfs.push(nbr);
        }
      }
    }

    // 平均边界质心
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    return output;
  }

  /**
   * @brief 判断单元格是否是一个新的前沿单元格(可发展成边界)
   */
  bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontierFlag)
  {
    // 检查单元格是否未知且未被标记为边界
    if (map[idx] != NO_INFORMATION || frontierFlag[idx])
    {
      return false;
    }

    // 边界单元格在4连通的邻域中至少应有一个空闲单元
    for (unsigned int nbr : nhood4(idx, *costmap))
    {
      if (map[nbr] == FREE_SPACE)
      {
        return true;
      }
    }

    return false;
  }

  /**
   * @brief 边界代价
   * @param frontier 需要计算的边界
   * @return 边界最终的代价
   */
  double FrontierSearch::frontierCost(const Frontier &frontier)
  {
    // potentialScale 到该边界距离的加权系数
    // gainScale 该边界大小的加权系数
    // costmap->getResolution() 地图的分辨率
    return (potentialScale * frontier.minDistance *
            costmap->getResolution()) -
           (gainScale * frontier.size * costmap->getResolution());
  }
}
