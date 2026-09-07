#include "../include/slam.h"

using namespace std;

SLAM::SLAM(QWidget *parent) : QWidget(parent),
                              qnode(NULL, NULL),
                              ui(new Ui::SLAM)
{
    ui->setupUi(this);
    initData();                                          // 初始化RVIZ的话题数据
    connections();                                       // 添加控件事件回响链接
    ros::param::get("/sebot_marking/pathXml", pathXml); // XML文件地址
}

SLAM::~SLAM()
{
    if (map_rviz_)
    {
        delete map_rviz_;
        map_rviz_ = nullptr;
    }
    delete ui;
}

/**
 * @brief SLAM::connections 控件事件回响链接
 */
void SLAM::connections()
{
    connect(menuAddTopic, SIGNAL(Topic_choose(QTreeWidgetItem *, QString)), this, SLOT(slot_choose_topic(QTreeWidgetItem *, QString))); // 绑定添加rviz话题信号
}

/**
 * @brief SLAM::initData 初始化数据
 */
void SLAM::initData()
{
    // RVIZ 话题控件数据
    m_mapRvizDisplays.insert("Axes", RVIZ_DISPLAY_AXES);
    m_mapRvizDisplays.insert("Camera", RVIZ_DISPLAY_CAMERA);
    m_mapRvizDisplays.insert("DepthCloud", RVIZ_DISPLAY_DEPTHCLOUD);
    m_mapRvizDisplays.insert("Effort", RVIZ_DISPLAY_EFFORT);
    m_mapRvizDisplays.insert("FluidPressure", RVIZ_DISPLAY_FLUIDPRESSURE);
    m_mapRvizDisplays.insert("Grid", RVIZ_DISPLAY_GRID);
    m_mapRvizDisplays.insert("GridCells", RVIZ_DISPLAY_GRIDCELLS);
    m_mapRvizDisplays.insert("Group", RVIZ_DISPLAY_GROUP);
    m_mapRvizDisplays.insert("Illuminance", RVIZ_DISPLAY_ILLUMINANCE);
    m_mapRvizDisplays.insert("Image", RVIZ_DISPLAY_IMAGE);
    m_mapRvizDisplays.insert("InterativerMarker", RVIZ_DISPLAY_INTERATIVEMARKER);
    m_mapRvizDisplays.insert("LaserScan", RVIZ_DISPLAY_LASERSCAN);
    m_mapRvizDisplays.insert("Map", RVIZ_DISPLAY_MAP);
    m_mapRvizDisplays.insert("Marker", RVIZ_DISPLAY_MARKER);
    m_mapRvizDisplays.insert("MarkerArray", RVIZ_DISPLAY_MARKERARRAY);
    m_mapRvizDisplays.insert("Odometry", RVIZ_DISPLAY_ODOMETRY);
    m_mapRvizDisplays.insert("Path", RVIZ_DISPLAY_PATH);
    m_mapRvizDisplays.insert("PointCloud", RVIZ_DISPLAY_POINTCLOUD);
    m_mapRvizDisplays.insert("PointCloud2", RVIZ_DISPLAY_POINTCLOUD2);
    m_mapRvizDisplays.insert("PointStamped", RVIZ_DISPLAY_POINTSTAMPED);
    m_mapRvizDisplays.insert("Polygon", RVIZ_DISPLAY_POLYGON);
    m_mapRvizDisplays.insert("Pose", RVIZ_DISPLAY_POSE);
    m_mapRvizDisplays.insert("PoseArray", RVIZ_DISPLAY_POSEARRAY);
    m_mapRvizDisplays.insert("PoseWithCovariance", RVIZ_DISPLAY_POSEWITHCOVARIANCE);
    m_mapRvizDisplays.insert("Range", RVIZ_DISPLAY_RANGE);
    m_mapRvizDisplays.insert("RelativeHumidity", RVIZ_DISPLAY_RELATIVEHUMIDITY);
    m_mapRvizDisplays.insert("RobotModel", RVIZ_DISPLAY_ROBOTMODEL);
    m_mapRvizDisplays.insert("TF", RVIZ_DISPLAY_TF);
    m_mapRvizDisplays.insert("Temperature", RVIZ_DISPLAY_TEMPERATURE);
    m_mapRvizDisplays.insert("WrenchStamped", RVIZ_DISPLAY_WRENCHSTAMPED);

    // UI数据初始化
    this->setWindowModality(Qt::ApplicationModal);

    menuAddTopic = new AddTopics(); // RVIZ话题菜单初始化

    // 读取本地配置
    QSettings settings("Qt-Ros Package", "cyrobot_rviz_tree");
    restoreGeometry(settings.value("geometry").toByteArray());
    // restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url", QString("http://192.168.1.13:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.13")).toString();

    // 读取快捷指令的setting
    QSettings quick_setting("quick_setting", "cyrobot_rviz_tree");
    QStringList ch_key = quick_setting.childKeys();

    // 模式初始化
    ui->button_savemap->setVisible(false);
    ui->button_set2DPose->setVisible(false);
    ui->button_setGoal->setVisible(true);
    ui->page_table->setVisible(true);
    ui->widget_goal1->setVisible(false);
    ui->widget_goal2->setVisible(false);
    ui->widget_goal3->setVisible(false);
    ui->widget_goal4->setVisible(false);
    ui->widget_goal5->setVisible(false);
    ui->widget_goal6->setVisible(false);
    ui->toolBox->setCurrentIndex(1);
    this->setWindowTitle("『Location』工厂定位");
    connect(&qnode, SIGNAL(sendMultiNaviGaols()), this, SLOT(getMultiNaviGaols())); // 链接槽函数

    ui->checkBox_table->setChecked(true);

    // 链接ROS-Master
    if (!qnode.initNavi())
    {
        QMessageBox::warning(nullptr, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes, QMessageBox::Yes);
        this->close(); // 关闭窗口
        return;
    }
    // 初始化rviz
    initRviz();
    sleep(1); // 等待ROS初始化完毕
    // process->start("roslaunch sebot_marking sebot_qt_multinavi.launch"); // 启动多点导航节点
    // 启动定时器
    //  idTimer = startTimer(1000);  //间隔 单位: 毫秒
}

#pragma region "RVIZ设置" {

/**
 * @brief SLAM::initRviz RVIZ监视器初始化
 */
void SLAM::initRviz()
{
    map_rviz_ = new QRviz(ui->verticalLayout_build_map, "qrviz");
    connect(map_rviz_, &QRviz::ReturnModelSignal, this, &SLAM::rvizGetModel);
    map_rviz_->GetDisplayTreeModel();
    QMap<QString, QVariant> namevalue;
    namevalue.insert("Line Style", "Billboards");
    namevalue.insert("Color", QColor(160, 160, 160));
    namevalue.insert("Plane Cell Count", 10);
    map_rviz_->DisplayInit(RVIZ_DISPLAY_GRID, "Grid", true, namevalue);

    ui->button_addTopic->setEnabled(true);
    ui->pushButton_rvizReadDisplaySet->setEnabled(true);
    ui->pushButton_rvizSaveDisplaySet->setEnabled(true);
    ui->pushButton_remove_topic->setEnabled(true);

    std::string path = ros::package::getPath("sebot_marking") + "/resources/location.yaml";
    map_rviz_->ReadDisplaySet(QString::fromStdString(path)); // 默认打开RVIZ设置
}

void SLAM::rvizGetModel(QAbstractItemModel *model)
{
    m_modelRvizDisplay = model;
    ui->treeView_rvizDisplayTree->setModel(model);
}

/**
 * @brief SLAM::rvizAddTopic 添加RIVZ话题
 */
void SLAM::on_button_addTopic_clicked()
{
    // 在父窗口中心打开
    QPoint globalPos = this->mapToGlobal(QPoint(0, 0));                    // 父窗口绝对坐标
    int x = globalPos.x() + (this->width() - menuAddTopic->width()) / 2;   // x坐标
    int y = globalPos.y() + (this->height() - menuAddTopic->height()) / 2; // y坐标
    menuAddTopic->move(x, y);                                              // 窗口移动
    menuAddTopic->show();
}

/**
 * @brief SLAM::rvizReadDisplaySet 导入RVIZ配置
 */
void SLAM::on_pushButton_rvizReadDisplaySet_clicked()
{
    if (map_rviz_ == nullptr)
    {
        return;
    }

    QString userPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation); // 打开本地文件存储地址
    QString userName = userPath.section("/", -1, -1);
    QString path = QFileDialog::getOpenFileName(this, "导入 RVIZ Display 配置", "/home/" + userName + "/", "YAML(*.yaml);;ALL(*.*)");
    if (!path.isEmpty())
    {
        map_rviz_->ReadDisplaySet(path);
    }
}

/**
 * @brief SLAM::rvizSaveDisplaySet 导出RVIZ配置
 */
void SLAM::on_pushButton_rvizSaveDisplaySet_clicked()
{
    if (map_rviz_ == nullptr)
    {
        return;
    }
    QString userPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation); // 打开本地文件存储地址
    QString userName = userPath.section("/", -1, -1);
    QString path = QFileDialog::getSaveFileName(this, "导出 RVIZ Display 配置", "/home/" + userName + "/", "YAML(*.yaml)");

    if (!path.isEmpty())
    {
        if (path.section('/', -1, -1).indexOf('.') < 0)
        {
            path = path + ".yaml";
        }
        map_rviz_->OutDisplaySet(path);
    }
}

/**
 * @brief SLAM::removeRvizTopic 删除RVIZ话题
 */
void SLAM::on_pushButton_remove_topic_clicked()
{
    if (ui->treeView_rvizDisplayTree->currentIndex().row() >= 0)
    {
        m_sRvizDisplayChooseName_ = ui->treeView_rvizDisplayTree->currentIndex().data().value<QString>();
        map_rviz_->RemoveDisplay(m_sRvizDisplayChooseName_);
        if (ui->treeView_rvizDisplayTree->currentIndex().row() >= 0)
        {
            on_treeView_rvizDisplayTree_clicked(ui->treeView_rvizDisplayTree->currentIndex());
        }
        else
        {
            m_sRvizDisplayChooseName_.clear();
        }
    }
    else
    {
        inform("请选择Display后再执行此操作");
    }
}

/**
 * @brief SLAM::on_pushButton_rename_topic_clicked 重命名RVIZ话题
 */
void SLAM::on_pushButton_rename_topic_clicked(bool checked)
{
    if (ui->treeView_rvizDisplayTree->currentIndex().row() < 0)
    {
        inform("请选择Display后再执行此操作");
        return;
    }
    QString dlgTitle = "重命名";
    QString txtlabel = "请输入名字：";
    QString defaultInupt = m_sRvizDisplayChooseName_;
    QLineEdit::EchoMode echoMode = QLineEdit::Normal;
    bool ok = false;
    QString newname = QInputDialog::getText(this, dlgTitle, txtlabel, echoMode, defaultInupt, &ok);
    if (ok && !newname.isEmpty())
    {
        if (newname != defaultInupt)
        {
            QString nosamename = JudgeDisplayNewName(newname);
            map_rviz_->RenameDisplay(defaultInupt, nosamename);
            m_sRvizDisplayChooseName_ = nosamename;
            if (nosamename != newname)
            {
                inform("命名重复！命名已自动更改为" + nosamename);
            }
        }
    }
    else if (ok)
    {
        inform("输入内容为空，重命名失败");
    }
}

/**
 * @brief SLAM::on_treeView_rvizDisplayTree_clicked 点击事件|刷新RVIZ节点信息
 * @param index
 */
void SLAM::on_treeView_rvizDisplayTree_clicked(const QModelIndex &index)
{
    m_sRvizDisplayChooseName_ = index.data().value<QString>();
    if (index.parent().row() == -1) // Display 的根节点
    {
        if (index.row() > 1)
        {
            ui->pushButton_remove_topic->setEnabled(true);
            ui->pushButton_rename_topic->setEnabled(true);
        }
        else
        {
            ui->pushButton_remove_topic->setEnabled(false);
            ui->pushButton_rename_topic->setEnabled(false);
        }
    }
    else
    {
        ui->pushButton_remove_topic->setEnabled(false);
        ui->pushButton_rename_topic->setEnabled(false);
    }
}

/**
 * @brief SLAM::slot_choose_topic 选中要添加的话题的槽函数
 * @param choose
 * @param name
 */
void SLAM::slot_choose_topic(QTreeWidgetItem *choose, QString name)
{
    QString ClassID = choose->text(0);

    name = JudgeDisplayNewName(name); // 检查重名

    qDebug() << "choose topic ClassID:" << ClassID << ", name:" << name;
    QMap<QString, QVariant> namevalue;
    namevalue.clear();
    map_rviz_->DisplayInit(m_mapRvizDisplays[ClassID], name, true, namevalue);
}

/**
 * @brief SLAM::JudgeDisplayNewName 检查重名
 * @param name
 * @return
 */
QString SLAM::JudgeDisplayNewName(QString name)
{
    if (m_modelRvizDisplay != nullptr)
    {
        bool same = true;
        while (same)
        {
            same = false;
            for (int i = 0; i < m_modelRvizDisplay->rowCount(); i++)
            {
                // m_sRvizDisplayChooseName = index.data().value<QString>();
                if (m_modelRvizDisplay->index(i, 0).data().value<QString>() == name)
                {
                    if (name.indexOf("_") != -1)
                    {
                        int num = name.section("_", -1, -1).toInt();
                        name = name.left(name.length() - name.section("_", -1, -1).length() - 1);
                        if (num <= 1)
                        {
                            num = 2;
                        }
                        else
                        {
                            num++;
                        }
                        name = name + "_" + QString::number(num);
                    }
                    else
                    {
                        name = name + "_2";
                    }
                    same = true;
                    break;
                }
            }
        }
    }
    return name;
}

/**
 * @brief SLAM::on_button_moveCamera_clicked 移动RVIZ图像中心
 */
void SLAM::on_button_moveCamera_clicked()
{
    map_rviz_->Set_MoveCamera();
}

/**
 * @brief SLAM::on_button_select_clicked 选择区域
 */
void SLAM::on_button_select_clicked()
{
    map_rviz_->Set_Select();
}

/**
 * @brief SLAM::on_button_set2DPose_clicked 设置机器人位置（重定位）
 */
void SLAM::on_button_set2DPose_clicked()
{
    map_rviz_->Set_Pos();
}

/**
 * @brief 设置导航终点
 */
void SLAM::on_button_setGoal_clicked()
{
    map_rviz_->Set_multiGoal();
}

/**
 * @brief 保存地图
 */
void SLAM::on_button_savemap_clicked()
{
    processSaveMap->start("rosrun map_server map_saver -f /root/map");
    inform("地图文件已保存:/root/map.yaml");
}

#pragma endregion }

/**info
 * @brief SLAM::inform 弹窗提示
 * @param info
 */
void SLAM::inform(QString info)
{
    QMessageBox m_r;
    m_r.setWindowTitle("提示");
    m_r.setText(info);
    m_r.exec();
}

/**
 * @brief 系统退出
 * @param event
 */
void SLAM::closeEvent(QCloseEvent *event)
{
    delete map_rviz_;
    map_rviz_ = nullptr;

    QProcess *processExit = new QProcess(this);
    processExit->start("killall map_server rplidarNode sebot_multinavigation amcl move_base"); // 杀进程
    sleep(1);
    process->close(); // 多点导航任务退出
    processSaveMap->close();
    processExit->close();
    sleep(1);
    this->close();
}

/**
 * @brief 通信获取到多点导航坐标
 */
void SLAM::getMultiNaviGaols()
{
    if (qnode.siteType == qnode.Location::SITE_TABLE) // 位置信息类型
    {
        ui->widget_goal1->setVisible(false);
        ui->widget_goal2->setVisible(false);
        ui->widget_goal3->setVisible(false);
        ui->widget_goal4->setVisible(false);
        ui->widget_goal5->setVisible(false);
        ui->widget_goal6->setVisible(false);

        for (int i = 0; i < qnode.gaolsMultiNavi.size(); i++)
        {
            QString goal = "P(";
            goal += QString::number(qnode.gaolsMultiNavi[i].pose.position.x) + ",";
            goal += QString::number(qnode.gaolsMultiNavi[i].pose.position.y) + ",";
            goal += QString::number(qnode.gaolsMultiNavi[i].pose.position.z) + ")";
            goal += "  O(";
            goal += QString::number(qnode.gaolsMultiNavi[i].pose.orientation.x) + ",";
            goal += QString::number(qnode.gaolsMultiNavi[i].pose.orientation.y) + ",";
            goal += QString::number(qnode.gaolsMultiNavi[i].pose.orientation.z) + ",";
            goal += QString::number(qnode.gaolsMultiNavi[i].pose.orientation.w) + ")";
            switch (i)
            {
            case 0:
                ui->widget_goal1->setVisible(true);
                ui->lineEdit_goal1->setText(goal);
                break;
            case 1:
                ui->widget_goal2->setVisible(true);
                ui->lineEdit_goal2->setText(goal);
                break;
            case 2:
                ui->widget_goal3->setVisible(true);
                ui->lineEdit_goal3->setText(goal);
                break;
            case 3:
                ui->widget_goal4->setVisible(true);
                ui->lineEdit_goal4->setText(goal);
                break;
            case 4:
                ui->widget_goal5->setVisible(true);
                ui->lineEdit_goal5->setText(goal);
                break;
            case 5:
                ui->widget_goal6->setVisible(true);
                ui->lineEdit_goal6->setText(goal);
                break;
            }
        }
    }
    else if (qnode.siteType == qnode.Location::SITE_SERVING) // 取件台
    {
        QString goal = "P(";
        goal += QString::number(qnode.gaolServing.pose.position.x) + ",";
        goal += QString::number(qnode.gaolServing.pose.position.y) + ",";
        goal += QString::number(qnode.gaolServing.pose.position.z) + ")";
        goal += "  O(";
        goal += QString::number(qnode.gaolServing.pose.orientation.x) + ",";
        goal += QString::number(qnode.gaolServing.pose.orientation.y) + ",";
        goal += QString::number(qnode.gaolServing.pose.orientation.z) + ",";
        goal += QString::number(qnode.gaolServing.pose.orientation.w) + ")";
        ui->lineEdit_serving->setText(goal);
    }
    else if (qnode.siteType == qnode.Location::SITE_ORIGIN) // 起始点
    {
        QString goal = "P(";
        goal += QString::number(qnode.gaolOrigin.pose.position.x) + ",";
        goal += QString::number(qnode.gaolOrigin.pose.position.y) + ",";
        goal += QString::number(qnode.gaolOrigin.pose.position.z) + ")";
        goal += "  O(";
        goal += QString::number(qnode.gaolOrigin.pose.orientation.x) + ",";
        goal += QString::number(qnode.gaolOrigin.pose.orientation.y) + ",";
        goal += QString::number(qnode.gaolOrigin.pose.orientation.z) + ",";
        goal += QString::number(qnode.gaolOrigin.pose.orientation.w) + ")";
        ui->lineEdit_origin->setText(goal);
    }
    else if (qnode.siteType == qnode.Location::SITE_COUNTER) // 结算区
    {
        QString goal = "P(";
        goal += QString::number(qnode.gaolCounter.pose.position.x) + ",";
        goal += QString::number(qnode.gaolCounter.pose.position.y) + ",";
        goal += QString::number(qnode.gaolCounter.pose.position.z) + ")";
        goal += "  O(";
        goal += QString::number(qnode.gaolCounter.pose.orientation.x) + ",";
        goal += QString::number(qnode.gaolCounter.pose.orientation.y) + ",";
        goal += QString::number(qnode.gaolCounter.pose.orientation.z) + ",";
        goal += QString::number(qnode.gaolCounter.pose.orientation.w) + ")";
        ui->lineEdit_counter->setText(goal);
    }
}

#pragma "删除目标点" {
void SLAM::on_button_deleteGoal1_clicked()
{
    ui->checkBox_table->setChecked(true);
    qnode.siteType = qnode.Location::SITE_TABLE;                  // 位置信息类型
    qnode.gaolsMultiNavi.erase(std::begin(qnode.gaolsMultiNavi)); // 删除Vector元素
    getMultiNaviGaols();
}

void SLAM::on_button_deleteGoal2_clicked()
{
    ui->checkBox_table->setChecked(true);
    qnode.siteType = qnode.Location::SITE_TABLE;                      // 位置信息类型
    qnode.gaolsMultiNavi.erase(std::begin(qnode.gaolsMultiNavi) + 1); // 删除Vector元素
    getMultiNaviGaols();
}

void SLAM::on_button_deleteGoal3_clicked()
{
    ui->checkBox_table->setChecked(true);
    qnode.siteType = qnode.Location::SITE_TABLE;                      // 位置信息类型
    qnode.gaolsMultiNavi.erase(std::begin(qnode.gaolsMultiNavi) + 2); // 删除Vector元素
    getMultiNaviGaols();
}

void SLAM::on_button_deleteGoal4_clicked()
{
    ui->checkBox_table->setChecked(true);
    qnode.siteType = qnode.Location::SITE_TABLE;                      // 位置信息类型
    qnode.gaolsMultiNavi.erase(std::begin(qnode.gaolsMultiNavi) + 3); // 删除Vector元素
    getMultiNaviGaols();
}

void SLAM::on_button_deleteGoal5_clicked()
{
    ui->checkBox_table->setChecked(true);
    qnode.siteType = qnode.Location::SITE_TABLE;                      // 位置信息类型
    qnode.gaolsMultiNavi.erase(std::begin(qnode.gaolsMultiNavi) + 4); // 删除Vector元素
    getMultiNaviGaols();
}

void SLAM::on_button_deleteGoal6_clicked()
{
    ui->checkBox_table->setChecked(true);
    qnode.siteType = qnode.Location::SITE_TABLE;                      // 位置信息类型
    qnode.gaolsMultiNavi.erase(std::begin(qnode.gaolsMultiNavi) + 5); // 删除Vector元素
    getMultiNaviGaols();
}
#pragma }

/**
 * @brief 保存信息
 */
void SLAM::on_button_save_clicked()
{
    if (qnode.gaolsMultiNavi.size() < 1)
    {
        QMessageBox::warning(nullptr, "警告！", "请设置[工作台]位置信息...", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    else if (ui->lineEdit_serving->text().toUtf8().length() < 1)
    {
        QMessageBox::warning(nullptr, "警告！", "请设置[取件台]位置信息...", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    else if (ui->lineEdit_origin->text().toUtf8().length() < 1)
    {
        QMessageBox::warning(nullptr, "警告！", "请设置[起始点]位置信息...", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    else if (ui->lineEdit_counter->text().toUtf8().length() < 1)
    {
        QMessageBox::warning(nullptr, "警告！", "请设置[结算区]位置信息...", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }

    write2Xml();
    QMessageBox::warning(nullptr, "提示!", "定位信息已保存至xml文件!!!", QMessageBox::Yes, QMessageBox::Yes);
}

/**
 * @brief 定时器中断：1s
 * @param ev
 */
void SLAM::timerEvent(QTimerEvent *event)
{
}

/**
 * @brief SLAM::选择工作台位置
 * @param arg1
 */
void SLAM::on_checkBox_table_stateChanged(int index)
{
    if (index == Qt::Checked)
    {
        ui->checkBox_serving->setChecked(false);
        ui->checkBox_origin->setChecked(false);
        ui->checkBox_counter->setChecked(false);

        qnode.siteType = qnode.Location::SITE_TABLE; // 位置信息类型
    }
}

/**
 * @brief SLAM::选择取件台位置
 * @param arg1
 */
void SLAM::on_checkBox_serving_stateChanged(int index)
{
    if (index == Qt::Checked)
    {
        ui->checkBox_table->setChecked(false);
        ui->checkBox_origin->setChecked(false);
        ui->checkBox_counter->setChecked(false);

        qnode.siteType = qnode.Location::SITE_SERVING; // 位置信息类型
    }
}

/**
 * @brief SLAM::选择起始点位置
 * @param arg1
 */
void SLAM::on_checkBox_origin_stateChanged(int index)
{
    if (index == Qt::Checked)
    {
        ui->checkBox_serving->setChecked(false);
        ui->checkBox_table->setChecked(false);
        ui->checkBox_counter->setChecked(false);

        qnode.siteType = qnode.Location::SITE_ORIGIN; // 位置信息类型
    }
}

/**
 * @brief SLAM::选择结算区位置
 * @param arg1
 */
void SLAM::on_checkBox_counter_stateChanged(int index)
{
    if (index == Qt::Checked)
    {
        ui->checkBox_serving->setChecked(false);
        ui->checkBox_origin->setChecked(false);
        ui->checkBox_table->setChecked(false);

        qnode.siteType = qnode.Location::SITE_COUNTER; // 位置信息类型
    }
}

/**
 * @brief 写Xml文件
 *
 */
void SLAM::write2Xml()
{
    TiXmlDocument *xmlFile = new TiXmlDocument; // xml文档指针
    // 文档格式声明
    TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "UTF-8", "yes");
    xmlFile->LinkEndChild(decl); // 写入文档
    // TiXmlElement父类的析构函数内自带delete，所以不用自己释放

    // 创建根元素
    TiXmlElement *rootElement = new TiXmlElement("location"); // 根元素
    xmlFile->LinkEndChild(rootElement);

    //[01] 工作台信息
    for (int i = 0; i < qnode.gaolsMultiNavi.size(); i++)
    {
        QString pos = QString::number(qnode.gaolsMultiNavi[i].pose.position.x) + " " + QString::number(qnode.gaolsMultiNavi[i].pose.position.y) + " " + QString::number(qnode.gaolsMultiNavi[i].pose.position.z);
        QString ori = QString::number(qnode.gaolsMultiNavi[i].pose.orientation.x) + " " + QString::number(qnode.gaolsMultiNavi[i].pose.orientation.y) + " " + QString::number(qnode.gaolsMultiNavi[i].pose.orientation.z) + " " + QString::number(qnode.gaolsMultiNavi[i].pose.orientation.w);

        // 创建元素
        TiXmlElement *tableElement = new TiXmlElement("table");         // 标签
        tableElement->SetAttribute("name", "工作台-" + to_string(i + 1)); // 添加属性
        rootElement->LinkEndChild(tableElement);                        // 父节点写入文档
        // 定位信息
        TiXmlElement *localElement = new TiXmlElement("local");
        localElement->SetAttribute("position", pos.toStdString());
        localElement->SetAttribute("orientation", ori.toStdString());
        tableElement->LinkEndChild(localElement);
    }

    //[02] 取件台信息
    QString position = QString::number(qnode.gaolServing.pose.position.x) + " " + QString::number(qnode.gaolServing.pose.position.y) + " " + QString::number(qnode.gaolServing.pose.position.z);
    QString orientation = QString::number(qnode.gaolServing.pose.orientation.x) + " " + QString::number(qnode.gaolServing.pose.orientation.y) + " " + QString::number(qnode.gaolServing.pose.orientation.z) + " " + QString::number(qnode.gaolServing.pose.orientation.w);
    // 创建元素
    TiXmlElement *serveElement = new TiXmlElement("serving"); // 标签
    serveElement->SetAttribute("name", "取件台");             // 添加属性
    // 定位信息
    TiXmlElement *localServeElement = new TiXmlElement("local");
    localServeElement->SetAttribute("position", position.toStdString());
    localServeElement->SetAttribute("orientation", orientation.toStdString());
    serveElement->LinkEndChild(localServeElement);
    rootElement->LinkEndChild(serveElement); // 父节点写入文档

    //[03] 起始点信息
    position = QString::number(qnode.gaolOrigin.pose.position.x) + " " + QString::number(qnode.gaolOrigin.pose.position.y) + " " + QString::number(qnode.gaolOrigin.pose.position.z);
    orientation = QString::number(qnode.gaolOrigin.pose.orientation.x) + " " + QString::number(qnode.gaolOrigin.pose.orientation.y) + " " + QString::number(qnode.gaolOrigin.pose.orientation.z) + " " + QString::number(qnode.gaolOrigin.pose.orientation.w);
    // 创建元素
    TiXmlElement *originElement = new TiXmlElement("origin"); // 标签
    originElement->SetAttribute("name", "起始点");            // 添加属性
    rootElement->LinkEndChild(originElement);                 // 父节点写入文档
    // 定位信息
    TiXmlElement *localOriElement = new TiXmlElement("local");
    localOriElement->SetAttribute("position", position.toStdString());
    localOriElement->SetAttribute("orientation", orientation.toStdString());
    originElement->LinkEndChild(localOriElement);

    //[04] 结算区信息
    position = QString::number(qnode.gaolCounter.pose.position.x) + " " + QString::number(qnode.gaolCounter.pose.position.y) + " " + QString::number(qnode.gaolCounter.pose.position.z);
    orientation = QString::number(qnode.gaolCounter.pose.orientation.x) + " " + QString::number(qnode.gaolCounter.pose.orientation.y) + " " + QString::number(qnode.gaolCounter.pose.orientation.z) + " " + QString::number(qnode.gaolCounter.pose.orientation.w);
    // 创建元素
    TiXmlElement *counterElement = new TiXmlElement("counter"); // 标签
    counterElement->SetAttribute("name", "结算区");             // 添加属性
    rootElement->LinkEndChild(counterElement);                  // 父节点写入文档
    // 定位信息
    TiXmlElement *localCouElement = new TiXmlElement("local");
    localCouElement->SetAttribute("position", position.toStdString());
    localCouElement->SetAttribute("orientation", orientation.toStdString());
    counterElement->LinkEndChild(localCouElement);

    // pathXml = ros::package::getPath("sebot_marking") + "/resources/location.xml";
    xmlFile->SaveFile(pathXml.c_str());
    delete xmlFile;
}
