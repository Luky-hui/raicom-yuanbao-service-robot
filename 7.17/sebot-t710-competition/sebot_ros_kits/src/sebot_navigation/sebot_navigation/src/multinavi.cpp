/**
 * @file multinavi.cpp
 * @author HC
 * @brief ROS机器人多点导航
 * @version 0.1
 * @date 2024/02/28 09:18:13
 * @copyright  :Copyright (c) 2023
 * @note 具体功能模块:
 */

#include "../include/multinavi.hpp"

const std::string strXmlFile = "/root/workspace/sebot-t710-competition/sebot_ros_kits/src/sebot_catering/res/location.xml";

MultiNavi::MultiNavi() : moveBaseAction("move_base", true), initialPoint(true)
{
    int timeW;
    ros::param::get("/multinavi/waitTime", timeW); // 接收导航等待时间
    waitTime = ros::Duration(timeW);               // 转换时间

    readXml(strXmlFile);
    // 构建定时器，定时判断是否达到目标点，以发送下一个目标
    timer_ = rosNode.createTimer(ros::Duration(0.3), &MultiNavi::callbackTimer, this); // 0.3s中断

    ROS_INFO("Starting success");
}
MultiNavi::~MultiNavi() {}

/**
 * @brief 读取 xml 文件
 * @param xmlPath 文件所在路径
 */
void MultiNavi::readXml(const std::string &xmlPath)
{
    if (xmlPath.empty())
    {
        ROS_ERROR_STREAM("Xml file does not exist!!!");
        ROS_ERROR_STREAM("Xml path:" + xmlPath);
        return;
    }

    // 1. 创建 xml 文档对象
    TiXmlDocument *fileData = new TiXmlDocument();

    // 2. 加载 xml 文件
    if (!fileData->LoadFile(xmlPath.c_str(), TIXML_ENCODING_UNKNOWN))
    {
        ROS_ERROR_STREAM("Can't load xml file!");
        return;
    }

    // 3.获取根节点
    TiXmlElement *root = fileData->RootElement();
    if (!root)
    {
        ROS_ERROR_STREAM("Failed to get root element!");
        return;
    }

    // 4.遍历根节点并写入容器
    for (TiXmlElement *child = root->FirstChildElement(); child; child = child->NextSiblingElement())
    {
        const char *elementName = child->Value();
        move_base_msgs::MoveBaseGoal goal;
        std::vector<double> positionDoubles;    // 用于储存转换为double类型的position数据
        std::vector<double> orientationDoubles; // 用于储存转换为double类型的orientation数据

        // 筛选节点类型
        if (std::strcmp(elementName, "table") == 0 || std::strcmp(elementName, "serving") == 0 ||
            std::strcmp(elementName, "origin") == 0 || std::strcmp(elementName, "counter") == 0)
        {
            const std::string positionAtt = child->FirstChildElement("local")->Attribute("position");       // 读取position属性
            const std::string orientationAtt = child->FirstChildElement("local")->Attribute("orientation"); // 读取orientation属性
            if (!positionAtt.empty() && !orientationAtt.empty())                                            // 如果属性都存在
            {
                positionDoubles = readDouble(positionAtt);       // 解析字符串并转换数据类型
                orientationDoubles = readDouble(orientationAtt); // 解析字符串并转换数据类型
                goal.target_pose.pose.position.x = positionDoubles[0];
                goal.target_pose.pose.position.y = positionDoubles[1];
                goal.target_pose.pose.position.z = positionDoubles[2];
                goal.target_pose.pose.orientation.x = orientationDoubles[0];
                goal.target_pose.pose.orientation.y = orientationDoubles[1];
                goal.target_pose.pose.orientation.z = orientationDoubles[2];
                goal.target_pose.pose.orientation.w = orientationDoubles[3];
                goal.target_pose.header.frame_id = "map";
                if (std::strcmp(elementName, "table") == 0 || std::strcmp(elementName, "serving") == 0 ||
                    std::strcmp(elementName, "counter") == 0)
                {
                    goal.target_pose.header.seq = index + 1;
                    gaolsMulti.push_back(goal); // 将导航点添加进容器
                    index++;
                }

                else if (std::strcmp(elementName, "origin") == 0)
                {
                    goal.target_pose.header.seq = 1;
                    gaolsMulti.emplace(gaolsMulti.begin(), goal); // 将起始点添加进容器
                    initialPoint = true;
                }
            }
        }
    }

    return;
}

/**
 * @brief 定时器回调函数
 * @param
 */
void MultiNavi::callbackTimer(const ros::TimerEvent &event)
{
    if (initialPoint)
    {
        sleep(5);
        initialPoint = false;
        targetPose = gaolsMulti.front();                        // 获取第一个元素作为起始点
        gaolsMulti.erase(gaolsMulti.begin());                   // 删除第一个元素
        targetPose.target_pose.header.stamp = ros::Time::now(); // 获取时间戳

        // 创建PoseWithCovariance消息
        geometry_msgs::PoseWithCovarianceStamped pose_msg;

        // 设置pose
        pose_msg.header.stamp = ros::Time::now(); // 当前时间戳
        pose_msg.header.frame_id = "map";         // 参考坐标系，通常是地图坐标系
        pose_msg.pose.pose.position = targetPose.target_pose.pose.position;
        pose_msg.pose.pose.orientation = targetPose.target_pose.pose.orientation;
        pose_msg.pose.covariance = {0.24765850579972618, 0.0003599694617748967, 0.0, 0.0, 0.0, 0.0, 0.0003599694617748955, 0.24290098927446058, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06716567243247433};

        // 发布pose消息到/amcl_pose话题
        amcl_pose_pub.publish(pose_msg);
        return;
    }

    // 判断是否到达目标点，如果成功，则发布下一个点
    if (moveBaseAction.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        timer = ros::Time::now(); // 记录当前时间
        return;
    }
    if (ros::Time::now() - timer > waitTime && !timer.is_zero())
    {
        targetPose = gaolsMulti.front();      // 获取第一个元素作为起始点
        gaolsMulti.erase(gaolsMulti.begin()); // 删除第一个元素
        // targetPose.target_pose.pose.position.x -= initial.x;     // 转换为相对于地图的坐标
        // targetPose.target_pose.pose.position.y -= initial.y;
        targetPose.target_pose.header.stamp = ros::Time::now(); // 获取时间戳
        std::cerr << targetPose << std::endl;
        if (!moveBaseAction.waitForServer(ros::Duration(60))) // 等待move_base服务
        {
            ROS_INFO("Can't connected to move base server");
        }
        moveBaseAction.sendGoal(targetPose, boost::bind(&MultiNavi::doneCb, this, _1, _2),
                                boost::bind(&MultiNavi::activeCb, this),
                                rosAction::SimpleFeedbackCallback());
    }
    return;
}

/**
 * @brief 解析字符串变为double
 * @param data 需要解析的字符串
 * @return 返回一个double容器
 */
std::vector<double> MultiNavi::readDouble(const std::string &data)
{
    std::string token;
    std::istringstream iss(data);
    std::vector<double> doubles;
    // 循环读取每一个数字
    while (std::getline(iss, token, ' ')) // 将字符串根据空格拆分开
    {
        double value;
        try
        {
            // 尝试将字符串转换为 double
            value = std::stod(token);
            doubles.push_back(value); // 将 double 值储存到容器
        }
        catch (const std::invalid_argument &e)
        {
            // 如果转换失败，打印错误信息
            std::cerr << "Invalid number format: " << token << std::endl;
        }
        catch (const std::out_of_range &e)
        {
            // 如果数值超出 double 类型的表示范围，打印错误信息
            std::cerr << "Number out of range: " << token << std::endl;
        }
    }
    return doubles;
}

void MultiNavi::doneCb(const actionlib::SimpleClientGoalState &state,
                       const move_base_msgs::MoveBaseResultConstPtr &result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("SUCCEEDED");
    }
}

void MultiNavi::activeCb()
{
    ROS_INFO("Goal Received");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multinavi");
    MultiNavi multiNavi;
    ros::spin();
    return 0;
}
