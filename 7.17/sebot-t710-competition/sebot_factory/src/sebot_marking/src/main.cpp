/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/slam.h"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    SLAM *slamWindow = nullptr; // SLAM窗口视图类

    slamWindow = new SLAM();
    slamWindow->setWindowModality(Qt::ApplicationModal); // 设置子界面为模态窗口
    slamWindow->showMaximized();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    while(1)
    {
        app.exec();
    }
    return app.exec();
}
