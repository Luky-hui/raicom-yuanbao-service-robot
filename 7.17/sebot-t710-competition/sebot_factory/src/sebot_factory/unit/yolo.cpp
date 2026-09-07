/**
 * @file yolo.cpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 基于PaddleDetection NPU/Yolov3 目标检测模型推理
 * @version 0.1
 * @date 2024-03-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <chrono>
#include <time.h>
#include "../include/detection.hpp"
#include "../include/tools.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sebot_yolo"); // ROS节点初始化
    float score = 0.5;                   // AI检测置信度
    ros::param::get("/sebot_yolo/score", score);
    string pathPkg = ros::package::getPath("sebot_factory");                         // 功能包文件根路径
    shared_ptr<Detection> detection = make_shared<Detection>(pathPkg + "/res/model"); // 初始化NPU及AI模型
    detection->score = score;                                                         // AI检测置信度

    // USB摄像头初始化

    std::string device;
    if (!getVideoDevice(VideoIndex::ASTRA_RGB, device))
        device = "/dev/deepCamera";
    VideoCapture capture(device); // 打开摄像头
    // VideoCapture capture(pathPkg + "/res/demo.mp4"); // 打开本地视频
    if (!capture.isOpened())
    {
        cout << "can not open video device " << endl;
        return 0;
    }
    capture.set(CAP_PROP_FRAME_WIDTH, 320);  // 设置图像的列
    capture.set(CAP_PROP_FRAME_HEIGHT, 240); // 设置图像的行

    Mat img;
    while (true)
    {
        if (!capture.read(img)) // 相机采集数据
            continue;

        // 启动AI推理
        detection->inference(img); // NPU推理
        detection->drawBox(img);   // 绘制目标框
        // 帧率计算
        static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
        auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
        printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime, 1000.0 / (startTime - preTime));
        preTime = startTime;

        imshow("img", img);
        waitKey(10);
    }

    capture.release();

    return 0;
}
