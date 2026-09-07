#ifndef __FRONTIER_SEARCH_H__
#define __FRONTIER_SEARCH_H__

#include <costmap_2d/costmap_2d.h>

namespace frontier_exploration
{
  /**
   * @brief 边界结构体
   *
   */
  struct Frontier
  {
    std::uint32_t size;                       // 边界大小
    double minDistance;                       // 最小距离
    double cost;                              // 代价
    geometry_msgs::Point initial;             // 初始点
    geometry_msgs::Point centroid;            // 质心点
    geometry_msgs::Point middle;              // 中点
    std::vector<geometry_msgs::Point> points; // 点列表
  };

  /**
   * @brief Thread-safe implementation of a frontier-search task for an input costmap.
   */
  class FrontierSearch
  {
  public:
    FrontierSearch() {}

    /**
     * @brief 搜索边界
     * @param costmap 在地图上进行搜索
     */
    FrontierSearch(costmap_2d::Costmap2D *costmap, double potential_scale, double gain_scale, double minFrontierSize);

    /**
     * @brief 运行搜索实现，从起始位置向外运行
     * @param position 初始位置
     * @return 边界列表
     */
    std::vector<Frontier> searchFrom(geometry_msgs::Point position);

  protected:
    /**
     * @brief 从初始单元开始，从有效相邻单元构建边界
     * @param initial_cell Index of cell to start frontier building
     * @param reference 用来计算位置的参考索引
     * @param frontier_flag 记录哪些单元格已被标记
     * @return 新边界
     */
    Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool> &frontier_flag);

    /**
     * @brief 评估候选单元是否是新边界的有效候选单元
     * @param idx 候选索引
     * @param frontier_flag 记录哪些单元格已被标记
     * @return true | false
     */
    bool isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag);

    /**
     * @brief 计算前沿代价
     * @details 代价由潜在成本和收益决定，潜在成本:到该边界的距离 收益:该边界的大小
     * @param frontier 需要计算代价的边界
     * @return 该边界的代价
     */
    double frontierCost(const Frontier &frontier);

  private:
    costmap_2d::Costmap2D *costmap;   // 代价地图数据
    unsigned char *map;               // 地图数据
    unsigned int sizeX, sizeY;        // 地图尺寸
    double potentialScale, gainScale; // 边界距离加权和大小加权
    double minFrontierSize;           // 最小边界大小
  };
}
#endif
