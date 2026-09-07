/**
 * @file arm.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 机械臂-UT单元测试
 * @version 0.1
 * @date 2024/03/15 15:33:44
 * @copyright  :Copyright (c) 2024
 * @note 具体功能模块:
 */

#include "../include/arm.hpp"
#include "../include/tools.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sebot_arm");

    // 机械臂初始化
    Talon talon;
    sleep(2);

    if (!talon.isOpen)
    {
        ROS_ERROR_STREAM("Talon-Uart open failed!!!");
        return 0;
    }
    talon.setActions(talon.ACTION_EXT); // 机械臂动作：伸展

    while (1)
    {
        ;
    }

    // talon.setJointEnable(talon.ArmJoint::ARM_JOINT_ALL, false); // 机械臂失能
    // sleep(1);
    // talon.setJointEnable(talon.ArmJoint::ARM_JOINT_ALL, false); // 机械臂失能
    // sleep(1);
    // talon.close(); // 关闭串口
}
