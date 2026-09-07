/**
 * @file order.cpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 需求确认-UT单元测试
 * @version 0.1
 * @date 2024-03-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "../src/confirm.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sebot_confirm");
    Confirm confirm;                                                 // 实例化需求确认类
    ros::param::get("/sebot_confirm/pidPoseKp", confirm.pidPose.kP); // 方向控制PID
    ros::param::get("/sebot_confirm/pidPoseKi", confirm.pidPose.kI);
    ros::param::get("/sebot_confirm/pidPoseKd", confirm.pidPose.kD);
    ros::param::get("/sebot_confirm/pidDisKp", confirm.pidDis.kP); // 距离控制PID
    ros::param::get("/sebot_confirm/pidDisKi", confirm.pidDis.kI);
    ros::param::get("/sebot_confirm/pidDisKd", confirm.pidDis.kD);
    ros::param::get("/sebot_confirm/distance", confirm.pidDis.ref);   // 机器人距离阈值：m（需要+激光雷达盲区20cm）
    ros::param::get("/sebot_picking/pidLocalKp", confirm.pidLocal.kP); // 位置控制PID
    ros::param::get("/sebot_picking/pidLocalKi", confirm.pidLocal.kI);
    ros::param::get("/sebot_picking/pidLocalKd", confirm.pidLocal.kD);
    ros::param::get("/sebot_confirm/debug", confirm.debug);

    ros::Rate loop_rate(20); // 设置ROS循环周期: Hz
    bool run = false;
    while (ros::ok())
    {
        if (!run)
            run = confirm.getConfirm();
        else
        {
            std::cout << "orders: ";
            for (int i = 0; i < confirm.orders.size(); i++)
            {
                std::cout << confirm.orders[i] << " ";
            }
            std::cout << "." << std::endl;
        }
        ros::spinOnce();   // 回调话题通信
        loop_rate.sleep(); // ROS线程周期控制
    }
    return 0;
}
