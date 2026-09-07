/**
 * @file summary.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 智能结算场景服务
 * @version 0.1
 * @date 2024/03/14 09:12:30
 * @copyright  :Copyright (c) 2024
 * @note 具体功能模块:
 */

#include "../include/tools.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

class Order
{
public:
    int tableId;            // 工作台编号
    vector<string> part;    // 工作台订单
};

class Summary
{
    private:
    /**
     * @brief 智能结算零件价格
     *
     */
    std::unordered_map<std::string, std::string> partNumber; // 零件价格字典

    /**
     * @brief 智能结算订单情况
     *
     */
    enum SummaryStep
    {
        SUMMARY_STEP_START = 0, // 任务开始
        SUMMARY_STEP_NONE,      // 没有订单
        SUMMARY_STEP_ONE,       // 一个订单
        SUMMARY_STEP_TWO,       // 二个订单
        SUMMARY_STEP_THREE,     // 三个订单
        SUMMARY_STEP_MORE,      // 更多订单
        SUMMARY_STEP_END        // 结束
    };

    SummaryStep payStep = SummaryStep::SUMMARY_STEP_START; // 智能结算任务流程
    
    public:
    string pathPkg;     // 功能包路径
    Mat imgB;           // 背景图片
    Mat imgT;           // 工作台图片
    Mat part;           // 零件图片
    Mat imgMoney;       // 金钱图片
    Mat imgMoneyD;      // 金钱(red)图片
    cv::Mat stitching;  // 图片缝合
    Summary()
    {
        partNumber[LABEL_AI_SCREW]  = "G111";                               // 螺钉的编号
        partNumber[LABEL_AI_NUT]    = "G112";                               // 螺母的编号
        partNumber[LABEL_AI_PCB]    = "G113";                               // 线路板的编号
        partNumber[LABEL_AI_BLOCK]  = "G114";                            // 端子排的编号
        partNumber[LABEL_AI_TAPE]   = "G115";                              // 绝缘胶带的编号
        pathPkg = ros::package::getPath("sebot_factory");           // 功能包文件根路径
        imgB = imread(pathPkg + "/res/image/background.png");       // 读取背景图片
        imgT = imread(pathPkg + "/res/image/table.png");            // 读取件桌图片
        imgMoney = imread(pathPkg + "/res/image/money.png");        // 读取人民币图片
        imgMoneyD = imread(pathPkg + "/res/image/money(red).png");  // 读取人民币(红)图片

        if (imgB.empty() || imgT.empty()) 
        {  
            ROS_INFO_STREAM("Could not open or find the base image!");
        }  
        
        cv::resize(imgB, imgB, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR); // 缩放背景图片
    };
    
    /**
     * @brief 获取订单信息
     * @param orders 全部订单信息
     */
    void getAll(vector<Order> orders)
    {
        switch (orders.size())
        {
        case 0:                                // 没有订单
            payStep = SummaryStep::SUMMARY_STEP_NONE;
            break;
        case 1:                                // 1个订单
            payStep = SummaryStep::SUMMARY_STEP_ONE;
            break;
        case 2:                                // 2个订单
            payStep = SummaryStep::SUMMARY_STEP_TWO;
            break;
        case 3:                                // 3个订单
            payStep = SummaryStep::SUMMARY_STEP_THREE;
            break;
        }
        if (orders.size() > 3 )
            payStep = SummaryStep::SUMMARY_STEP_MORE;  // 更多订单
    }

    /**
     * @brief 一个订单任务
     * @param orders 全部订单信息
     */
    void orderOne(vector<Order> orders,Mat imgTable,int num)
    {
        Point text1(150, 325);      // 书写金额
        Point text2(606, 405);      // 书写金额(红)
        Point text3(400, 350);      // 书写编号
        
        std::string numberStr = "NO." + to_string(orders[num-1].tableId);
        putText(imgTable, numberStr, text3, FONT_HERSHEY_SIMPLEX, 4, Scalar(0, 0, 255), 10);          // 书写编号

        if (orders[num-1].part.size() != 1)
        {
            ROS_ERROR_STREAM("Error : "+ to_string(num) + " num");
            return;
        }

        if (orders[num-1].part[0] == "")
        {
            ROS_ERROR_STREAM("Error : "+ to_string(num) + " num");
            return;
        }

        part = imread(pathPkg + "/res/image/" + orders[num-1].part[0] +".png");     // 读取零件图片 
        stitching = imgTable(cv::Rect(61, 63, part.cols, part.rows));
        part.copyTo(stitching);  

        putText(imgTable, partNumber[orders[num-1].part[0]], text1, FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 5);  // 书写金额
    }

    /**
     * @brief 两个订单任务
     * @param orders 全部订单信息
     */
    void orderTwo(vector<Order> orders,Mat imgTable,int num)
    {
        Point text1(100, 450);         // 左侧零件的价格
        Point text2(400, 450);         // 右侧零件的价格
        Point text3(290, 175);         // 总价格
        Point text4(150, 150);         // 书写编号
        
        std::string numberStr = "NO." + to_string(orders[num-1].tableId);
        putText(imgTable, numberStr, text4, FONT_HERSHEY_SIMPLEX, 4, Scalar(0, 0, 255), 10);      // 书写编号

        if (orders[num-1].part.size() != 2)
        {
            ROS_ERROR_STREAM("Error : "+ to_string(num) + " num");
            return;
        }

        if (orders[num-1].part[0] == "" || orders[num-1].part[1] == "")
        {
            ROS_ERROR_STREAM("Error : "+ to_string(num) + " num");
            return;
        }
        
        part = imread(pathPkg + "/res/image/" + orders[num-1].part[0] +".png");   // 读取零件图片 
        cv::resize(part, part, cv::Size(120, 120), 0, 0, cv::INTER_LINEAR);       // 缩放工作台图片
        stitching = imgTable(cv::Rect(90, 260, part.cols, part.rows));            // 绘制零件
        part.copyTo(stitching);  
        part = imread(pathPkg + "/res/image/" + orders[num-1].part[1] +".png");   // 读取零件图片 
        cv::resize(part, part, cv::Size(120, 120), 0, 0, cv::INTER_LINEAR);       // 缩放工作台图片
        stitching = imgTable(cv::Rect(400, 260, part.cols, part.rows));           // 绘制零件
        part.copyTo(stitching); 
        
        putText(imgTable, partNumber[orders[num-1].part[0]], text1, FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 5);  // 书写金额
        putText(imgTable, partNumber[orders[num-1].part[1]], text2, FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 5);  // 书写金额
    }

    /**
     * @brief 三个订单任务
     * @param orders 全部订单信息
     */
    void orderThree(vector<Order> orders,Mat imgTable,int num)
    {
        Point text1(70, 450);         // 左侧零件的价格
        Point text2(290, 450);         // 中间零件的价格
        Point text3(500, 450);         // 右侧零件的价格
        Point text4(290, 175);         // 总价格
        Point text5(150, 150);         // 书写编号

        std::string numberStr = "NO." + to_string(orders[num-1].tableId);
        putText(imgTable, numberStr,text5, FONT_HERSHEY_SIMPLEX, 4, Scalar(0, 0, 255), 10);      // 书写编号 

        if (orders[num-1].part.size() != 3)
        {
            ROS_ERROR_STREAM("Error : "+ to_string(num) + " num");
            return;
        }

        if (orders[num-1].part[0] == "" || orders[num-1].part[1] == "" || orders[num-1].part[2] == "")
        {
            ROS_ERROR_STREAM("Error : "+ to_string(num) + " num");
            return;
        }
        
        part = imread(pathPkg + "/res/image/" + orders[num-1].part[0] +".png");   // 读取零件图片 
        cv::resize(part, part, cv::Size(120, 120), 0, 0, cv::INTER_LINEAR);       // 缩放工作台图片
        stitching = imgTable(cv::Rect(76, 274, part.cols, part.rows));            // 绘制零件
        part.copyTo(stitching);  

        part = imread(pathPkg + "/res/image/" + orders[num-1].part[1] +".png");   // 读取零件图片 
        cv::resize(part, part, cv::Size(120, 120), 0, 0, cv::INTER_LINEAR);       // 缩放工作台图片
        stitching = imgTable(cv::Rect(298, 274, part.cols, part.rows));           // 绘制零件
        part.copyTo(stitching); 

        part = imread(pathPkg + "/res/image/" + orders[num-1].part[2] +".png");   // 读取零件图片 
        cv::resize(part, part, cv::Size(120, 120), 0, 0, cv::INTER_LINEAR);       // 缩放工作台图片
        stitching = imgTable(cv::Rect(520, 274, part.cols, part.rows));           // 绘制零件
        part.copyTo(stitching); 
        
        putText(imgTable, partNumber[orders[num-1].part[0]], text1, FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 5);  // 书写金额
        putText(imgTable, partNumber[orders[num-1].part[1]], text2, FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 5);    // 书写金额
        putText(imgTable, partNumber[orders[num-1].part[2]], text3, FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 5);  // 书写金额
        
    }


    /**
     * @brief 一个工作台
     * @param orders 全部订单信息
     */
    void tableOne(vector<Order> orders)
    {
        cv::resize(imgT, imgT, cv::Size(728, 488), 0, 0, cv::INTER_LINEAR); // 缩放工作台图片
        switch(orders[0].part.size())
        {
        case 0:
            payStep = SummaryStep::SUMMARY_STEP_NONE;  // 结束
            return;
        case 1:  
            orderOne(orders,imgT,1);
            break;
        case 2:
            orderTwo(orders,imgT,1);
            break;
        case 3:
            orderThree(orders,imgT,1);
            break;
        }
        stitching = imgB(cv::Rect(596, 530, imgT.cols, imgT.rows)); // 将工作台图片放到背景中
        imgT.copyTo(stitching); 
    }

    /**
     * @brief 两个工作台
     * @param orders 全部订单信息
     */
    void tableTwo(vector<Order> orders)
    {
        cv::resize(imgT, imgT, cv::Size(728, 488), 0, 0, cv::INTER_LINEAR); // 缩放工作台图片
        for(int n = 0; n < orders.size(); n++)
        {
            cv::Mat imgCopy = imgT.clone();
            switch(orders[n].part.size())
            {
            case 0:
                ROS_ERROR_STREAM("Error : "+ to_string(n+1) + " num");
                break;
            case 1:  
                orderOne(orders,imgCopy,n+1);
                break;
            case 2:
                orderTwo(orders,imgCopy,n+1);
                break;
            case 3:
                orderThree(orders,imgCopy,n+1);
                break;
            }
            stitching = imgB(cv::Rect(167+n*865, 530, imgCopy.cols, imgCopy.rows)); // 将工作台图片放到背景中
            imgCopy.copyTo(stitching); 
        }
    }

    /**
     * @brief 三个工作台
     * @param orders 全部订单信息
     */
    void tableThree(vector<Order> orders)
    {
        cv::resize(imgT, imgT, cv::Size(728, 488), 0, 0, cv::INTER_LINEAR); // 缩放工作台图片
        for(int n = 0; n < orders.size(); n++)
        {
            cv::Mat imgCopy = imgT.clone();
            switch(orders[n].part.size())
            {
            case 0:
                ROS_ERROR_STREAM("Error : "+ to_string(n+1) + " num");
                break;
            case 1:  
                orderOne(orders,imgCopy,n+1);
                break;
            case 2:
                orderTwo(orders,imgCopy,n+1);
                break;
            case 3:
                orderThree(orders,imgCopy,n+1);
                break;
            }

            cv::resize(imgCopy, imgCopy, cv::Size(581, 392), 0, 0, cv::INTER_LINEAR); // 缩放工作台图片

            stitching = imgB(cv::Rect(30+n*611, 600, imgCopy.cols, imgCopy.rows)); // 将工作台图片放到背景中
            imgCopy.copyTo(stitching); 
        }
    }

    /**
     * @brief 更多工作台
     * @param orders 全部订单信息
     */
    void tableMore(vector<Order> orders)
    {
        cv::resize(imgT, imgT, cv::Size(728, 488), 0, 0, cv::INTER_LINEAR); // 缩放工作台图片

        static int number = 0;
        for(int n = number; n < 3 + number; n++)
        {
            cv::Mat imgCopy = imgT.clone();
            switch(orders[n].part.size())
            {
            case 0:
                ROS_ERROR_STREAM("Error : "+ to_string(n+1) + " num");
                break;
            case 1:  
                orderOne(orders,imgCopy,n+1);
                break;
            case 2:
                orderTwo(orders,imgCopy,n+1);
                break;
            case 3:
                orderThree(orders,imgCopy,n+1);
                break;
            }

            cv::resize(imgCopy, imgCopy, cv::Size(581, 392), 0, 0, cv::INTER_LINEAR); // 缩放工作台图片
            
            stitching = imgB(cv::Rect(30+(n-number)*611, 600, imgCopy.cols, imgCopy.rows)); // 将工作台图片放到背景中
            imgCopy.copyTo(stitching); 
        }
        if ((orders.size() - 3) == number)
            number = 0;     // 循环一轮
        else
            number++;       // 向后移动
    }

    /**
     * @brief 显示订单信息
     * @param orders 全部订单信息
     */
    bool showOrder(vector<Order> orders)
    {
        switch(payStep)
        {
        case SummaryStep::SUMMARY_STEP_START: 
            getAll(orders);         // 获取订单信息
            return false;  

        case SummaryStep::SUMMARY_STEP_NONE: 
            imgB = imread(pathPkg + "/res/image/None.png"); // 覆盖背景图片
            cv::resize(imgB, imgB, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR); // 缩放背景图片
            payStep = SummaryStep::SUMMARY_STEP_END;  // 结束
            return false;  
        case SummaryStep::SUMMARY_STEP_ONE:
            payStep = SummaryStep::SUMMARY_STEP_END;  // 结束
            tableOne(orders);                 // 处理单个工作台
            return false;
        case SummaryStep::SUMMARY_STEP_TWO:
            payStep = SummaryStep::SUMMARY_STEP_END;  // 结束
            tableTwo(orders);                 // 处理两个工作台
            return false;
        case SummaryStep::SUMMARY_STEP_THREE:
            payStep = SummaryStep::SUMMARY_STEP_END;  // 结束
            tableThree(orders);               // 处理三个工作台
            return false;
        case SummaryStep::SUMMARY_STEP_MORE:
            tableMore(orders);                // 处理三个工作台
            imshow("Result", imgB);           // 显示结果 
            waitKey(2000);  
            return false;
        case SummaryStep::SUMMARY_STEP_END:
            imshow("Result", imgB);           // 显示结果  
            waitKey(0);  
            return true;  
        }
    }
};
