/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/qnode.hpp"

using namespace std;

QNode::QNode(int argc, char **argv) : init_argc(argc),
                                      init_argv(argv)
{
  //    读取topic的设置
  QSettings topic_setting("topic_setting", "monitor");
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

/**
 * @brief ROS主线程
 */
void QNode::run()
{
  ros::Rate loop_rate(30);
  // 当当前节点没有关闭时
  while (ros::ok())
  {
    // 调用消息处理回调函数
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/**
 * @brief 首页初始化
 * @return
 */
bool QNode::initMain()
{
  ros::init(init_argc, init_argv, "checkin");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();

  ros::NodeHandle rosNode;

  start();
  return true;
}

/**
 * @brief 导航讲解初始化
 * @return
 */
bool QNode::initNavi()
{
  ros::init(init_argc, init_argv, "navigation");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();

  ros::NodeHandle rosNode;

  subMultiNavi = rosNode.subscribe("/multiGoal", 1000, &QNode::callbackMultiGoal, this); // 多点导航目标数据接收
  start();
  return true;
}

/**
 * @brief 多点导航数据接收回响函数
 * @param msg
 */
void QNode::callbackMultiGoal(const geometry_msgs::PoseStamped &goal)
{
  if (siteType == Location::SITE_TABLE) // 工作台
  {
    if (gaolsMultiNavi.size() >= 6)
    {
      QMessageBox::warning(nullptr, "警告！", "工作台位置最多可设置6个,请确认位置信息!!!", QMessageBox::Yes, QMessageBox::Yes);
      return;
    }
    gaolsMultiNavi.push_back(goal); // 新增导航点
  }
  else if (siteType == Location::SITE_SERVING) // 取件台
  {
    gaolServing = goal;
  }
  else if (siteType == Location::SITE_ORIGIN) // 起始点
  {
    gaolOrigin = goal;
  }
  else if (siteType == Location::SITE_COUNTER) // 结算区
  {
    gaolCounter = goal;
  }
  emit sendMultiNaviGaols(); // 传递数据给UI
}

/**
 * @brief
 */
void QNode::disinit()
{
  if (ros::isStarted())
  {
    ROS_INFO("ROS will shutdown");
    ros::shutdown();
    ros::waitForShutdown();
  }
  this->exit();
}

/**
 * @brief 发布导航目标点信息
 * @param frame
 * @param x
 * @param y
 * @param z
 * @param w
 */
void QNode::setNaviGoal(QString frame, double x, double y, double z, double w)
{
  geometry_msgs::PoseStamped goal;
  // 设置frame
  goal.header.frame_id = frame.toStdString();
  // 设置时刻
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0;
  goal.pose.orientation.z = z;
  goal.pose.orientation.w = w;
  pubGoal.publish(goal);
  ros::spinOnce();
}
