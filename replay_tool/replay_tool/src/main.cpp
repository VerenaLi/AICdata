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
#include "../include/replay_tool/main_window.hpp"
#include <QApplication>
//#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#include <ros/ros.h>
#include <QProcess>

#include <sensor_msgs/Imu.h>

int main(int argc, char *argv[])
{
      ros::init(argc, argv, "replay_tool");
      ros::NodeHandle nh;
      QApplication a(argc, argv);
      MainWindow w;
      w.RosInit(nh);
      w.show();
      return a.exec();


}
