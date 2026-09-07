/**
 * @file bfs.h
 * @author HC
 * @brief 代价地图中的广度优先搜索算法
 * @version 0.1
 * @date 2024/02/01 17:20:24
 * @copyright  :Copyright (c) 2023
 * @note 具体功能模块:
 */

#ifndef __COSTMAP_TOOLS__
#define __COSTMAP_TOOLS__

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>

namespace frontier_exploration
{
  /**
   * @brief 确定一个单元格的四连通邻域(上下左右四个方向)
   * edges
   * @param idx 初始单元格索引
   * @param costmap 代价地图数据
   * @return 相邻单元格的索引
   */
  std::vector<unsigned int> nhood4(unsigned int idx, const costmap_2d::Costmap2D &costmap)
  {

    std::vector<unsigned int> out; // 初始化单元格列表

    unsigned int sizeX = costmap.getSizeInCellsX(), // 获取地图大小
        sizeY = costmap.getSizeInCellsY();

    if (idx > sizeX * sizeY - 1) // 不计算地图最外侧的相邻单元格
    {
      ROS_WARN("Evaluating nhood for offmap point");
      return out;
    }

    if (idx % sizeX > 0) // 检查该索引是否为当前行的第一个单元格
      out.push_back(idx - 1);

    if (idx % sizeX < sizeX - 1) // 检查该索引是否为当前行的最后一个单元格
      out.push_back(idx + 1);

    if (idx >= sizeX)
      out.push_back(idx - sizeX); // 检查该索引是否为第一行的单元格

    if (idx < sizeX * (sizeY - 1)) // 检查该索引是否为最后一行的单元格
      out.push_back(idx + sizeX);

    return out;
  }

  /**
   * @brief 确定一个单元格的八连通邻域(九宫格)
   * @param idx 初始单元格索引
   * @param costmap 代价地图数据
   * @return 相邻单元格的索引
   */
  std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D &costmap)
  {
    // 先搜索索引单元格上下左右四个方向的单元格
    std::vector<unsigned int> out = nhood4(idx, costmap);

    unsigned int sizeX = costmap.getSizeInCellsX(), // 获取地图大小
        sizeY = costmap.getSizeInCellsY();

    if (idx > sizeX * sizeY - 1) // 不计算地图最外侧的相邻单元格
      return out;

    if (idx % sizeX > 0 && idx >= sizeX)
      out.push_back(idx - 1 - sizeX);

    if (idx % sizeX > 0 && idx < sizeX * (sizeY - 1))
      out.push_back(idx - 1 + sizeX);

    if (idx % sizeX < sizeX - 1 && idx >= sizeX)
      out.push_back(idx + 1 - sizeX);

    if (idx % sizeX < sizeX - 1 && idx < sizeX * (sizeY - 1))
      out.push_back(idx + 1 + sizeX);

    return out;
  }

  /**
   * @brief 查找与指定值最近的单元格
   * @param result 定位单元索引
   * @param start 搜索的初始单元格
   * @param val 要搜索的指定值
   * @param costmap 地图数据
   * @return 如果找到具有请求值的单元格，则为True
   */
  bool nearestCell(unsigned int &result, unsigned int start, unsigned char val,
                   const costmap_2d::Costmap2D &costmap)
  {
    const unsigned char *map = costmap.getCharMap();
    const unsigned int sizeX = costmap.getSizeInCellsX(), // 获取地图横向长度
        sizeY = costmap.getSizeInCellsY();                // 获取地图纵向长度

    if (start >= sizeX * sizeY) // 数据保护, 如果初始单元格不在地图内
      return false;

    // 初始化广度优先搜索算法
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(sizeX * sizeY, false);

    // 将初始单元格作为广度优先搜索算法的初始点
    bfs.push(start);
    visited_flag[start] = true; // 标记为已遍历

    // 当队列不为空时继续执行
    while (!bfs.empty())
    {
      unsigned int idx = bfs.front(); // 每次将队列前端的一个单元格作为索引
      bfs.pop();                      // 移除队列末端的一个单元格

      // 如果找到值正确的单元格则返回
      if (map[idx] == val) // 判断该索引单元格的代价值是否等于目标值
      {
        result = idx;
        return true;
      }

      // 遍历所有相邻的未访问单元格
      for (unsigned nbr : nhood8(idx, costmap))
      {
        if (!visited_flag[nbr])
        { // 如果相邻单元格未被标记，将其加入队列并标记
          bfs.push(nbr);
          visited_flag[nbr] = true;
        }
      }
    }

    return false;
  }
}
#endif
