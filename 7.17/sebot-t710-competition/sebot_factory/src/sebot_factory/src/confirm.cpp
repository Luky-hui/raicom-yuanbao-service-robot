#pragma once
/**
 * @file confirm.cpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 需求确认场景类
 * @version 0.1
 * @date 2024-03-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <geometry_msgs/Twist.h>   // 速控发布类
#include <sensor_msgs/LaserScan.h> // 订阅激光雷达数据
#include <chrono>
#include "../include/detection.hpp"
#include "../include/tools.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

using namespace cv;
using namespace std;

class Confirm
{
private:
    ros::NodeHandle rosNode;         // ROS节点类
    shared_ptr<Detection> detection; // AI目标检测类实例化指针
    VideoCapture capture;            // 摄像头实例化类
    ros::Subscriber subLaser;        // 订阅激光雷达话题
    ros::Publisher pubCmd;           // 发布速控话题
    long counter = 0;                // 计数器
    vector<PredictResult> partSearched;

    /**
     * @brief 需求确认任务流程
     *
     */
    enum ConfirmStep
    {
        CONFIRM_STEP_START = 0, // 任务开始
        CONFIRM_STEP_POSE,      // 机器人姿态校正
        CONFIRM_STEP_PART,      // 搜索订单下单
        CONFIRM_STEP_END        // 任务结束
    };
    ConfirmStep confirmStep = ConfirmStep::CONFIRM_STEP_START; // 需求确认任务流程

    /**
     * @brief 激光雷达类
     *
     */
    class Rplidar
    {
    public:
        float angle = 1.57;           // 激光雷达探测角度（机器人前方/rad）
        float angleIncrement = 0.087; // 间隔取点度数rad(≈5°)
        vector<float> rangesLeft;     // 选取左侧距离数据
        vector<float> rangesRight;    // 选取右侧距离数据
    };
    Rplidar rplidar; // 激光雷达类

    /**
     * @brief 接收激光雷达数据
     * @note angle_min: -3.12413907051  angle_max: 3.14159274101    angle_increment: 0.0174532923847
     *      time_increment: 1.80087170065e-07   scan_time: 6.46512926323e-05    range_min: 0.193000003695
     *      range_max: 8.0
     * @param msg
     */
    void getLaser(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        rplidar.rangesLeft.clear();
        rplidar.rangesRight.clear();

        if (msg->ranges.size() > 0 && msg->angle_increment > 0)
        {
            if (rplidar.angleIncrement < msg->angle_increment)
                rplidar.angleIncrement = msg->angle_increment;
            int numLaser = rplidar.angle / rplidar.angleIncrement;    // 截取的激光点数 (90)
            int steps = msg->ranges.size();                           // 激光雷达总步长
            int step = rplidar.angleIncrement / msg->angle_increment; // 间隔取点步长

            if (steps > numLaser * step)
            {
                for (int i = 0; i < numLaser / 2; i++)
                {
                    if (msg->ranges[i * step] > 0.2 && msg->ranges[i * step] < 1.0)
                        rplidar.rangesLeft.push_back(msg->ranges[i * step]);
                    if (msg->ranges[steps - i * step - 1] > 0.2 && msg->ranges[steps - i * step - 1] < 1.0)
                        rplidar.rangesRight.push_back(msg->ranges[steps - i * step - 1]);
                }
            }
        }
    }

    /**
     * @brief 获取容器数据中值
     *
     * @param stream
     * @return float
     */
    float getMedian(vector<float> stream)
    {
        // sort(stream.begin(), stream.end()); // 排序
        // int size = stream.size();
        // if (size & 1)                 // 位运算判断是否是奇数
        //     return stream[size >> 1]; // 奇数
        // else
        //     return (stream[size >> 1] + stream[(size - 1) >> 1]) / 2; // 偶数个
        sort(stream.begin(), stream.end()); // 排序
        int size = stream.size()/3;
        if (size & 1)                 // 位运算判断是否是奇数
            return stream[size]*1.414; // 奇数
        else
            return (stream[size] + stream[size + 1]) *1.414/ 2; // 偶数个
    }

    /**
     * @brief  机器人运动控制
     *
     * @param linearX X轴线速度
     * @param linearY Y轴线速度
     * @param angular Z轴角速度
     */
    void robotCtrl(float linearX, float linearY, float angular)
    {
        geometry_msgs::Twist msgCmd; // 速控话题数据
        msgCmd.linear.x = linearX;
        msgCmd.linear.y = linearY;
        msgCmd.angular.z = angular;
        pubCmd.publish(msgCmd); // 发布速控话题
    }

    /**
     * @brief 机器人姿态控制（距离+方向）
     *
     */
    void poseControl()
    {
        float linearX = 0.0, linearY = 0.0, angular = 0.0; // 速控话题数据

        if (rplidar.rangesLeft.size() >= 3 && rplidar.rangesRight.size() >= 3)
        {
            //[01] 方向控制
            if (pidPose.enable)
            {
                pidPose.feedBack = getMedian(rplidar.rangesLeft) - getMedian(rplidar.rangesRight); // ±0.3
                pidController(pidPose);                                                            // 姿态PID控制器
                angular = pidPose.out;
            }

            //[02] 距离控制
            if (pidDis.enable)
            {
                if (pidDis.ref < 0.2) // 激光雷达盲区距离
                    pidDis.ref = 0.2;
                pidDis.feedBack = (getMedian(rplidar.rangesLeft) + getMedian(rplidar.rangesRight)) / 2; // ±0.3
                pidController(pidDis);
                linearX = -pidDis.out;
            }

            // 位置控制
            if (pidLocal.enable)
            {
                pidController(pidLocal);
                linearY = pidLocal.out;
                // ROS_ERROR_STREAM("error:" + to_string(pidLocal.feedBack) + "  out:" + to_string(linearY));
            }

            robotCtrl(linearX, linearY, angular); // 机器人运动控制
        }
    }

    /**
     * @brief 搜索订单位置
     *
     * @param  零件AI标签
     * @return int 返回控制中心：center
     */
    int searchSignBoard()
    {
        int center = COLSIMAGE;
        Mat img;
        if (!capture.read(img)) // 相机采集数据
            return center;

        // 启动AI推理
        detection->score = score;
        detection->inference(img); // NPU推理

        vector<PredictResult> results;
        for (int i = 0; i < detection->results.size(); i++)
        {
            if (detection->results[i].label == LABEL_AI_ORDER &&
                detection->results[i].width < COLSIMAGE / 3 &&
                detection->results[i].height < ROWSIMAGE / 2) // 限制AI框大小，滤波
                results.push_back(detection->results[i]);
        }
        PredictResult result;
        result.score = 0;
        result.height = 0;
        result.width = 0;
        result.x = 0;
        result.y = 0;
        for (int i = 0; i < results.size(); i++)
        {
            if (results[i].score >= result.score)
                result = results[i];
        }

        // 控制中心求取
        if (result.label == LABEL_AI_ORDER)
            center = result.x + result.width / 2 - COLSIMAGE / 2;

        if (debug) // 调试模式
        {
            // 绘制目标框
            auto score = std::to_string(result.score);
            int pointY = result.y - 20;
            if (pointY < 0)
                pointY = 0;
            cv::Rect rectText(result.x, pointY, result.width, 20);
            cv::rectangle(img, rectText, cv::Scalar(0, 255, 0), -1);
            std::string label_name = result.label + " [" + score.substr(0, score.find(".") + 2) + "]";
            cv::Rect rect(result.x, result.y, result.width, result.height);
            cv::rectangle(img, rect, cv::Scalar(0, 255, 0), 1);
            cv::putText(img, label_name, Point(result.x, result.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);

            imshow("img", img);
            waitKey(1);
        }

        return center;
    }

    /**
     * @brief 搜索订单内容（零件）
     *
     * @return vector<PredictResult>
     */
    vector<PredictResult> searchParts()
    {
        vector<PredictResult> parts;

        Mat img;
        if (!capture.read(img)) // 相机采集数据
            return parts;

        // 启动AI推理
        detection->score = score;
        detection->inference(img); // NPU推理

        //[01] 优先检索订单牌的位置
        vector<PredictResult> results;
        for (int i = 0; i < detection->results.size(); i++)
        {
            if (detection->results[i].label == LABEL_AI_ORDER &&
                detection->results[i].width < COLSIMAGE / 3 &&
                detection->results[i].width > COLSIMAGE / 10 &&
                detection->results[i].height < ROWSIMAGE / 2) // 限制AI框大小，滤波
                results.push_back(detection->results[i]);
        }
        PredictResult order;
        order.score = 0;
        order.height = 0;
        order.width = 0;
        order.x = 0;
        order.y = 0;

        for (int i = 0; i < results.size(); i++)
        {
            if ((results[i].y + results[i].height / 2) >= (order.y + order.height / 2)) // 选择图像最底端的目标物
                order = results[i];
        }

        if (order.label != LABEL_AI_ORDER) // 未检索订单
            return parts;

        //[02] 开始检索有效零件信息（每种零件唯一）
        PredictResult nut, screw, pcb, block, tape; // 临时检索数据
        nut.score = 0;
        screw.score = 0;
        pcb.score = 0;
        block.score = 0;
        tape.score = 0;
        
        int number = 0;         // 当前订单数量
        std::vector<int> axisY; // 记录Y轴坐标数据
        for (int i = 0; i < detection->results.size(); i++)
        {
            int x = detection->results[i].x + detection->results[i].width / 2; // 目标物坐标
            int y = detection->results[i].y + detection->results[i].height / 2;
            if (x >= order.x && x <= (order.x + order.width) && y >= order.y && y <= (order.y + order.height)) // 目标位置必须在订单框中
            {
                
                // 统计订单数量
                axisY.push_back(y); // 用于计算订单数量

                // 更新零件信息（唯一性）
                if (detection->results[i].label == LABEL_AI_NUT && detection->results[i].score >= nut.score) // 螺母
                    nut = detection->results[i];
                if (detection->results[i].label == LABEL_AI_SCREW && detection->results[i].score >= screw.score)  // 螺钉
                    screw = detection->results[i];
                if (detection->results[i].label == LABEL_AI_PCB && detection->results[i].score >= pcb.score)  // 电路板
                    pcb = detection->results[i];
                if (detection->results[i].label == LABEL_AI_BLOCK && detection->results[i].score >= block.score)      // 端子排
                    block = detection->results[i];
                if (detection->results[i].label == LABEL_AI_TAPE && detection->results[i].score >= tape.score)   // 绝缘胶带
                    tape = detection->results[i];
            }
        }

        std::vector<int> result;  // 存储过滤后的结果
        for (int value : axisY) {
            bool toAdd = true;  // 标记该元素是否应添加
            for (int existingValue : result) {
                if (std::abs(value - existingValue) <= 10) {
                    toAdd = false;
                    break;  // 差值不超过 15，跳过当前值
                }
            }
            if (toAdd) {
                result.push_back(value);  // 保留符合条件的元素
            }
        }

        // 生成订单信息
        if (nut.label == LABEL_AI_NUT)
            parts.push_back(nut);
        if (screw.label == LABEL_AI_SCREW)
            parts.push_back(screw);
        if (pcb.label == LABEL_AI_PCB)
            parts.push_back(pcb);
        if (block.label == LABEL_AI_BLOCK)
            parts.push_back(block);
        if (tape.label == LABEL_AI_TAPE)
            parts.push_back(tape);

        // 按照Y轴中心点坐标排序
        std::sort(parts.begin(), parts.end(), [](const PredictResult& a, const PredictResult& b) {
            return (a.y + a.height / 2) < (b.y + b.height / 2); 
        });

        if (debug) // 调试模式
        {
            for (int i = 0; i < detection->results.size(); i++)
            {
                // 绘制目标框
                auto score = std::to_string(detection->results[i].score);
                int pointY = detection->results[i].y - 20;
                if (pointY < 0)
                    pointY = 0;
                cv::Rect rectText(detection->results[i].x, pointY, detection->results[i].width, 20);
                cv::rectangle(img, rectText, cv::Scalar(0, 255, 0), -1);
                std::string label_name = detection->results[i].label + " [" + score.substr(0, score.find(".") + 2) + "]";
                cv::Rect rect(detection->results[i].x, detection->results[i].y, detection->results[i].width, detection->results[i].height);
                cv::rectangle(img, rect, cv::Scalar(0, 255, 0), 1);
                cv::putText(img, label_name, Point(detection->results[i].x, detection->results[i].y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);
            }
            imshow("img", img);
            waitKey(1);
        }
        if (result.size() > partsCount)
            partsCount = result.size();

        return parts;
    }

public:
    Pid pidPose;           // PID姿态控制器：方向（角度）
    Pid pidDis;            // PID姿态控制器：距离（X轴）
    Pid pidLocal;          // PID姿态控制器：位置（Y轴）
    bool debug = false;    // 调试使能：AI绘制+显示
    float score = 0.4;     // AI置信度
    vector<string> orders; // 订单信息
    int partsCount = 0;    // 订单零件数量

    Confirm()
    {
        string pathPkg = ros::package::getPath("sebot_factory");   // 功能包文件根路径
        detection = make_shared<Detection>(pathPkg + "/res/model"); // 初始化NPU及AI模型
        detection->score = 0.4;                                     // AI检测置信度

        // 摄像头初始化
        std::string device;
        if (!getVideoDevice(VideoIndex::ASTRA_RGB, device))
            device = "/dev/deepCamera";
        capture = VideoCapture(device,CAP_V4L2); // 打开摄像头
        if (!capture.isOpened())
        {
            cout << "can not open video device " << endl;
            return;
        }
        capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像的列
        capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像的行

        // ROS节点类
        subLaser = rosNode.subscribe("/scan", 1, &Confirm::getLaser, this); // 订阅激光雷达话题
        pubCmd = rosNode.advertise<geometry_msgs::Twist>("/cmd_vel", 5);     // 速控话题发布器

        // PID控制器初始化
        pidPose.enable = true;
        pidDis.enable = true;
        pidDis.ref = 0.3;       // 机器人距离控制：m（激光雷达盲区20cm+10cm距离）
        pidDis.deadline = 0.05; // 机器人距离控制死区：m
        pidDis.outMax = 0.2;    // 限制输出速度：m/s
        pidLocal.deadline = 30; // 图像控制中心死区
        pidLocal.outMax = 0.12; // 限制输出速度：m/s
    };
    ~Confirm()
    {
        robotCtrl(0, 0, 0); // 机器人运动控制
        capture.release();
    };

    /**
     * @brief 需求确认（综合场景任务）
     *
     * @return true 返回任务完成状态
     * @return false
     */
    bool getConfirm()
    {
        static int count = 0; // AI检测计数器
        std::unordered_map<std::string, int> partCount;// 记录每种零件的出现次数

        switch (confirmStep)
        {
        case ConfirmStep::CONFIRM_STEP_START: // 任务开始
            counter = getSystTime();
            confirmStep = ConfirmStep::CONFIRM_STEP_POSE; // 切换下一个任务流程
            count = 0;                              // AI检测计数器
            break;

        case ConfirmStep::CONFIRM_STEP_POSE: // 机器人姿态校正
        {
            int center = searchSignBoard(); // 搜索订单位置
            if (center < COLSIMAGE)     // 检测到有效的目标零件
            {
                pidLocal.enable = true; // 使能位置PID控制
                pidLocal.feedBack = center;
            }
            else
                pidLocal.enable = false;

            // 姿态校正状态检测
            if ((abs(pidDis.feedBack - pidDis.ref) < 0.1 &&
                 abs(pidPose.feedBack) < 0.1) ||
                (getSystTime() - counter) > 2000) // 姿态调制至误差范围or超时
            {
                pidLocal.enable = false; // 禁止横向移动
                counter = getSystTime();
                confirmStep = ConfirmStep::CONFIRM_STEP_PART; // 切换下一个任务流程
            }
            poseControl(); // 机器人姿态控制（距离+方向+位置）
        }
        break;

        case ConfirmStep::CONFIRM_STEP_PART: // 搜索订单下单
            if (getSystTime() - counter > 200)
            {
                counter = getSystTime();
                
                vector<PredictResult> parts = searchParts(); // 搜索订单内容（零件）
                count++;

                std::cout << "Current sampled parts: ";
                for (int i = 0; i < parts.size(); i++)
                {
                    partSearched.push_back(parts[i]);
                    std::cout << parts[i].label << " ";
                }
                std::cout << "." << std::endl;

                // 统计当前采样周期内的零件出现次数
                for (const auto& part : partSearched) {
                    partCount[part.label]++; // 更新全局计数器
                }

            }
            if (count > 4) // 采样时间6x300ms = 3s
            {
                PredictResult nut, screw, pcb, block, tape; // 临时检索数据
                nut.score = 0;
                screw.score = 0;
                pcb.score = 0;
                block.score = 0;
                tape.score = 0;

                for (int i = 0; i < partSearched.size(); i++)
                {
                    // 更新零件信息（唯一性，按照最高得分进行确认）
                    if (partSearched[i].label == LABEL_AI_NUT && partSearched[i].score >= nut.score) // 螺母
                        nut = partSearched[i];
                    if (partSearched[i].label == LABEL_AI_SCREW && partSearched[i].score >= screw.score) // 螺钉
                        screw = partSearched[i];
                    if (partSearched[i].label == LABEL_AI_PCB && partSearched[i].score >= pcb.score) // 电路板
                        pcb = partSearched[i];
                    if (partSearched[i].label == LABEL_AI_BLOCK && partSearched[i].score >= block.score) // 端子排
                        block = partSearched[i];
                    if (partSearched[i].label == LABEL_AI_TAPE && partSearched[i].score >= tape.score) // 绝缘胶带
                        tape = partSearched[i];
                }

                // 将符合采样次数要求（>=3次）的零件放入临时容器进行位置排序
                std::vector<PredictResult> validParts;
                if (partCount[LABEL_AI_NUT] >= 2) validParts.push_back(nut);
                if (partCount[LABEL_AI_SCREW] >= 2) validParts.push_back(screw);
                if (partCount[LABEL_AI_PCB] >= 2) validParts.push_back(pcb);
                if (partCount[LABEL_AI_BLOCK] >= 2) validParts.push_back(block);
                if (partCount[LABEL_AI_TAPE] >= 2) validParts.push_back(tape);

                // 按照 Y 轴中心点从上往下排序
                std::sort(validParts.begin(), validParts.end(), [](const PredictResult& a, const PredictResult& b) {
                    return (a.y + a.height / 2) < (b.y + b.height / 2); 
                });
                
                // 输出统计结果并填充最终的 orders 列表
                std::cout << "\n--- Part Result (Sorted by Y-axis Top to Bottom) ---" << std::endl;
                orders.clear(); 
                for (const auto& part : validParts) {
                    orders.push_back(part.label);
                    std::cout << "Label: " << part.label << " (Detected " << partCount[part.label] << " times)" << std::endl;
                }
                std::cout << "----------------------------------\n" << std::endl;


                // std::vector<std::pair<std::string, int>> sortedPartCount(partCount.begin(), partCount.end());
                // // 按出现次数从高到低排序
                // std::sort(sortedPartCount.begin(), sortedPartCount.end(), 
                // [](const auto& a, const auto& b) {
                //     return a.second > b.second; // 降序排列
                // });

                // // 输出统计结果
                // std::cout << "\n--- Part Occurrence Statistics ---" << std::endl;
                // int i = 0;
                // for (const auto& pair : sortedPartCount) {
                //     i++;
                //     if (i> partsCount)
                //         break;
                //     std::cout << "Label " << pair.first << ": " << pair.second << " times" << std::endl;
                //     if (pair.second >=3)
                //         orders.push_back(pair.first);
                // }
                // std::cout << "----------------------------------\n" << std::endl;
                
                robotCtrl(0, 0, 0);                    // 机器人运动控制
                confirmStep = ConfirmStep::CONFIRM_STEP_END; // 切换下一个任务流程
                ROS_ERROR_STREAM("----------- AI SIZE:" + to_string(orders.size()));
            }
            else
                poseControl(); // 机器人姿态控制（距离+方向）
            break;

        case ConfirmStep::CONFIRM_STEP_END: // 任务结束
            robotCtrl(0, 0, 0);         // 机器人运动控制
            break;
        }

        if (confirmStep == ConfirmStep::CONFIRM_STEP_END) // 返回自主下单任务状态
            return true;
        else
            return false;
    }
};
