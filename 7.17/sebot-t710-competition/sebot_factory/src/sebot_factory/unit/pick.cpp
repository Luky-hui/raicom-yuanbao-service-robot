/**
 * @file pick.cpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 智能取件场景
 * @version 0.1
 * @date 2024-03-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "../src/picking.cpp"
#include <signal.h>
#include "../include/tools.hpp"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sebot_picking");
    Picking picking; // 实例化需求确认类
    // signal(SIGINT, exitSignal); // 程序退出信号
    std::string food = LABEL_AI_NUT; // 默认抓取螺钉
    // 机械臂初始化
    Talon talon;
    sleep(1);

    if (!talon.isOpen)
    {
        ROS_ERROR_STREAM("Talon-Uart open failed!!!");
        return 0;
    }

    int sencePick = 1;                                               // 1：抓取零件场景 | 2：放置零件场景 | 3：机械爪跟随零件控制
    ros::param::get("/sebot_picking/pidPoseKp", picking.pidPose.kP); // 方向控制PID
    ros::param::get("/sebot_picking/pidPoseKi", picking.pidPose.kI);
    ros::param::get("/sebot_picking/pidPoseKd", picking.pidPose.kD);
    ros::param::get("/sebot_picking/pidDisKp", picking.pidDis.kP); // 距离控制PID
    ros::param::get("/sebot_picking/pidDisKi", picking.pidDis.kI);
    ros::param::get("/sebot_picking/pidDisKd", picking.pidDis.kD);
    ros::param::get("/sebot_picking/pidLocalKp", picking.pidLocal.kP); // 位置控制PID
    ros::param::get("/sebot_picking/pidLocalKi", picking.pidLocal.kI);
    ros::param::get("/sebot_picking/pidLocalKd", picking.pidLocal.kD);
    ros::param::get("/sebot_picking/pidClawXKp", picking.pidClawX.kP); // 机械爪PID
    ros::param::get("/sebot_picking/pidClawXKi", picking.pidClawX.kI);
    ros::param::get("/sebot_picking/pidClawXKd", picking.pidClawX.kD);
    ros::param::get("/sebot_picking/pidClawYKp", picking.pidClawY.kP); // 机械爪PID
    ros::param::get("/sebot_picking/pidClawYKi", picking.pidClawY.kI);
    ros::param::get("/sebot_picking/pidClawYKd", picking.pidClawY.kD);
    ros::param::get("/sebot_picking/debug", picking.debug);
    ros::param::get("/sebot_picking/disClaw", picking.disClaw);     // 机械爪夹取零件距离：m（需要+激光雷达盲区20cm）
    ros::param::get("/sebot_picking/disPick", picking.disPick);     // 机械臂抓取零件距离：m（需要+激光雷达盲区20cm）
    ros::param::get("/sebot_picking/disSearch", picking.disSearch); // AI搜索零件距离：m（需要+激光雷达盲区20cm）
    ros::param::get("/sebot_picking/sencePick", sencePick);
    ros::param::get("/sebot_picking/food", food);

    ros::Rate loop_rate(30); // 设置ROS循环周期: Hz
    bool run = false;

    if (sencePick == 3)
        picking.setCamera(false);
    sleep(1);
    while (ros::ok())
    {
        if (sencePick == 1) // 抓取零件场景
        {
            if (!run)
                run = picking.pickupSomething(food);
        }
        else if (sencePick == 2) // 放置零件场景
        {
            picking.placePart = 2;
            if (!run)
                run = picking.putdownSomething(food);
        }
        else if (sencePick == 3) // 机械爪跟随零件控制
        {
            picking.pickupSomething(LABEL_AI_BLOCK);
            picking.pickStep = picking.PICK_STEP_AIM;
        }

        ros::spinOnce();   // 回调话题通信
        loop_rate.sleep(); // ROS线程周期控制
    }

    // picking.setJointEnable(false); // 机械臂失能

    return 0;
}