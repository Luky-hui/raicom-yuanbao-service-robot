/**
 * @file /include/monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MONITOR_QNODE_HPP_
#define MONITOR_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QLabel>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>
#include <iostream>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <QDebug>
#include <QMessageBox>
#include <stdio.h>
#include <stdlib.h>
#include <ros/package.h>

class QNode : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief 位置信息类型
     */
    enum Location
    {
        SITE_TABLE = 0, // 工作台
        SITE_SERVING,   // 取件台
        SITE_ORIGIN,    // 起始点
        SITE_COUNTER    // 结算区
    };
    Location siteType = Location::SITE_TABLE; // 位置信息类型

    void run();
    QNode(int argc, char **argv);
    virtual ~QNode();
    bool initMain();
    bool initSetting();
    bool initSlam();
    bool initFollow();
    bool initDelivery();
    bool initCheckin();
    bool initNavi();
    bool init(const std::string &master_url, const std::string &host_url);
    void disinit();
    void setNaviGoal(QString frame, double x, double y, double z, double w);
    void robotControl(float speedLineX, float speedLineY, float speedAngle);
    void speechRecognition();
    void faceRegist(QString name);
    void setServoThreshold(uint8_t chanel, uint16_t threshold);
    void getServoThreshold();
    void setFollowGoal();
    void robotReset();
    std::vector<geometry_msgs::PoseStamped> gaolsMultiNavi; // 工作台位置信息
    geometry_msgs::PoseStamped gaolServing;                 // 取件台位置信息
    geometry_msgs::PoseStamped gaolOrigin;                  // 起始点位置信息
    geometry_msgs::PoseStamped gaolCounter;                 // 结算区位置信息

Q_SIGNALS:
    void sendMultiNaviGaols();

public slots:

private:
    int init_argc;
    char **init_argv;
    ros::Subscriber subMultiNavi; // 多点导航目标点
    ros::Publisher pubGoal;       // 发布导航目标
    ros::Publisher pubCmdvel;     // 发布机器人速度话题
    ros::Publisher pubMutilGoals; // 发送多点导航目标
    ros::Publisher pubCommand;

    void callbackMultiGoal(const geometry_msgs::PoseStamped &goal);
};

#endif
