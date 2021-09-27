#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"


#include <QMainWindow>
#include <iostream>
#include <QMainWindow>
#include <QThread>
#include <QVector>
#include <QMutex>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QProcess>
#include <QThread>
#include <QErrorMessage>
#include <QCloseEvent>
#include <QInputDialog>
#include <signal.h>
#include <algorithm>
#include <dirent.h>
#include <ctime>
#include <chrono>
#include <string.h>
#include <ros/ros.h>
using namespace std;



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    ros::NodeHandle nh_;
    void RosInit(ros::NodeHandle &n);

    void run();
    void Time_Stamp(quint64 stamp);


    bool Check_play_flag();



private:
    QMutex mutex;
    Publishers *ros_pub;
    Ui::MainWindow *ui;

    bool play_flag;
    bool pause_flag;
    bool loop_flag;
    QString load_path;
    QString speed_str;



    int slider_checker;
    int slider_value;


signals:
    void setThreadFinished(bool);

private slots:
  void slotDoubleSpinbox_slider();
  void slotslider_DoubleSpinBox();
  void play();
  void Load_file_path();
  void SetStamp(quint64 stamp);
  void SliderPressed();

  void PlaySpeedChange(double value);
  void SliderValueChange(int value);

  void SliderValueApply();

  void rangechange();
  void Restart();

  void Saveconfig();//点击打开文件要运行的事件

  void Loadconfig();


};
