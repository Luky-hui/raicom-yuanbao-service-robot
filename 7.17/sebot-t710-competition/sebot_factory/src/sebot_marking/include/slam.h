#ifndef SLAM_H
#define SLAM_H

#include <QDateTime>
#include <QWidget>
#include <QSettings>
#include <QFileDialog>
#include <QStandardPaths>
#include <QLineEdit>
#include <QInputDialog>
#include "ui_slam.h"
#include "qrviz.hpp"
#include "qnode.hpp"
#include "addtopics.h"
#include <QTimer>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <QProcess>
#include <stdio.h>
#include <tinyxml.h>
#include <iostream>

namespace Ui
{
    class SLAM;
}

class SLAM : public QWidget
{
    Q_OBJECT

public:
    explicit SLAM(QWidget *parent = nullptr);
    ~SLAM();
    void initRviz();                             // RVIZ 视图初始化
    void inform(QString info);                   // 弹窗提示
    virtual void timerEvent(QTimerEvent *event); // 重写定时器的事件   虚函数 子类重写父类的虚函数
    std::string pathXml = "/root/location.xml";

private:
    Ui::SLAM *ui;
    QRviz *map_rviz_ = nullptr; // RVIZ视图类
    QAbstractItemModel *m_modelRvizDisplay;
    QMap<QString, QString> m_mapRvizDisplays;
    QNode qnode;                       // ROS节点类
    AddTopics *menuAddTopic = nullptr; // RVIZ 话题菜单类
    QString m_sRvizDisplayChooseName_; // RVIZ选择的话题
    void connections();
    void initData();
    void rvizGetModel(QAbstractItemModel *model);
    QString JudgeDisplayNewName(QString name); // 检查重名
    int idTimer;                               // 定时器的唯一标示
    int counterStartup = 0;                    // 功能启动计数器
    QProcess *process = new QProcess(this);
    QProcess *processSaveMap = new QProcess(this);
    void write2Xml();

protected:
    void closeEvent(QCloseEvent *event);

public slots:
    void slot_choose_topic(QTreeWidgetItem *choose, QString name);
    void getMultiNaviGaols();

private slots:
    void on_treeView_rvizDisplayTree_clicked(const QModelIndex &index);
    void on_pushButton_rename_topic_clicked(bool checked);
    void on_pushButton_rvizSaveDisplaySet_clicked();
    void on_pushButton_rvizReadDisplaySet_clicked();
    void on_button_addTopic_clicked();
    void on_pushButton_remove_topic_clicked();
    void on_button_moveCamera_clicked();
    void on_button_select_clicked();
    void on_button_set2DPose_clicked();
    void on_button_setGoal_clicked();
    void on_button_savemap_clicked();
    void on_button_deleteGoal1_clicked();
    void on_button_deleteGoal2_clicked();
    void on_button_deleteGoal3_clicked();
    void on_button_deleteGoal4_clicked();
    void on_button_deleteGoal5_clicked();
    void on_button_deleteGoal6_clicked();
    void on_button_save_clicked();
    void on_checkBox_table_stateChanged(int index);
    void on_checkBox_serving_stateChanged(int index);
    void on_checkBox_origin_stateChanged(int index);
    void on_checkBox_counter_stateChanged(int index);
};

#endif // SLAM_H
