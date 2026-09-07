#pragma once
/**
 * @file util.hpp
 * @author HC
 * @brief 公共方法类
 * @version 0.1
 * @date 2023-09-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <unistd.h> //系统类

using namespace std;

/**
 * @brief 32位数据内存对齐/联合体
 *
 */
typedef union
{
    uint8_t buff[4];
    float float32;
    int int32;
} Bit32Union;

/**
 * @brief 16位数据内存对齐/联合体
 *
 */
typedef union
{
    uint8_t buff[2];
    int int16;
    uint16_t uint16;
} Bit16Union;

/**
 * @brief odom点位信息
 *
 */
struct Point
{
public:
    float x;
    float y;

    Point()
    {
        x = 0.0;
        y = 0.0;
    };
    Point(const float xx, const float yy) : x(xx), y(yy){};
};
