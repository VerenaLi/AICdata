
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/replay_tool/main_window.hpp"

#include "ui_main_window.h"
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <ros/ros.h>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ros_pub = new Publishers(this, &mutex);
    ui->setupUi(this);
    ros_pub->start();
//    setWindowIcon(QIcon(":/images/icon.png"));
    play_flag = false;
    loop_flag = false;
    slider_checker = false;



    connect(ui->save_config,&QAction::triggered,this,&MainWindow::Saveconfig);
    connect(ui->load_config,&QAction::triggered,this,&MainWindow::Loadconfig);



    connect(ros_pub, SIGNAL(StampShow(quint64)), this, SLOT(SetStamp(quint64)));

    connect(ros_pub, SIGNAL(changerange()), this, SLOT(rangechange()));





//play
   connect(ros_pub, SIGNAL(StartSignal()), this, SLOT(play()));

   connect(ui->playButton,SIGNAL(pressed()),this,SLOT(play()));

   connect(ui->restartButton, SIGNAL(pressed()), this, SLOT(Restart()));

//load
     connect(ui->loadButton, SIGNAL(pressed()), this, SLOT(Load_file_path()));

//quit
     connect(ui->quitButton, &QPushButton::pressed, [this]() {
         QApplication *app;
         app->quit();
     });




    //speed
    connect(ui->doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(slotDoubleSpinbox_slider()));
    connect(ui->pSlider, SIGNAL(valueChanged(int)), this, SLOT(slotslider_DoubleSpinBox()));
    connect(ui->doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(PlaySpeedChange(double)));

    ui->doubleSpinBox->setRange(0.01,20.0);
    ui->doubleSpinBox->setValue(1.0);
    ui->doubleSpinBox->setSingleStep(0.01);//doubleSpinBox time
    ui->pSlider->setRange(0,200);



    connect(ui->horizontalSlider, SIGNAL(sliderPressed()), this, SLOT(SliderPressed()));//jidutiao
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChange(int)));
    connect(ui->horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(SliderValueApply()));

    ui->horizontalSlider->setValue(0);
    slider_value = 0;

}




MainWindow::~MainWindow()
{
    emit setThreadFinished(true); //Tell the thread to finish
    delete ui;
    ros_pub->quit();
    if(!ros_pub->wait(500)) //Wait until it actually has terminated (max. 3 sec)
    {
        ros_pub->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        ros_pub->wait(); //We have to wait again here!
    }
}

void MainWindow::RosInit(ros::NodeHandle &n)
{
    ros_pub->ros_init(n);
}






void MainWindow::Load_file_path() //点击load后加载路径，传路径参数给ros_pub，唤醒Ready函数
{
    ui->imu_checkBox->setEnabled(true);
    ui->gnss_checkBox->setEnabled(true);
    ui->velodyne_16_checkBox->setEnabled(true);
    ui->velodyne_32_checkBox->setEnabled(true);
    ui->bfs_checkBox->setEnabled(true);
    ui->stereo_checkBox->setEnabled(true);

    play_flag = false;
    ros_pub->play_flag_ = false;
    this->ui->playButton->setText(QString::fromStdString("Play"));

    QFileDialog path;

    load_path = path.getExistingDirectory();
    if ( load_path != "")
    {
       ros_pub->load_path_ = load_path.toStdString();
    }else{
        load_path = QString::fromStdString(ros_pub->load_path_);
    }

    Check_play_flag();

    ros_pub->Ready();

    this->ui->data_path->setText(load_path);

}





void MainWindow::Time_Stamp(quint64 stamp)//ros_pub传递时间戳给界面显示
{
    this->ui->time_stamp_label->setText(QString::number(stamp));
}

bool MainWindow::Check_play_flag()//检查界面checkbox并传递标志位给ros_pub
{
    if(ui->loop->isChecked()){
      loop_flag = true;
    }else{
      loop_flag = false;
    }
    if(ui->imu_checkBox ->isChecked()){
      ros_pub->imu_play_flag_ = true;
    }else{
      ros_pub->imu_play_flag_ = false;
    }

    if(ui->gnss_checkBox ->isChecked()){
      ros_pub->gnss_play_flag_ = true;
    }else{
      ros_pub->gnss_play_flag_ = false;
    }

    if(ui->velodyne_16_checkBox ->isChecked()){
      ros_pub->velodyne_16_play_flag_ = true;
    }else{
      ros_pub->velodyne_16_play_flag_ = false;
    }

    if(ui->velodyne_32_checkBox ->isChecked()){
      ros_pub->velodyne_32_play_flag_ = true;
    }else{
      ros_pub->velodyne_32_play_flag_ = false;
    }

    if(ui->stereo_checkBox ->isChecked()){
      ros_pub->stereo_play_flag_ = true;
    }else{
      ros_pub->stereo_play_flag_ = false;
    }

    if(ui->bfs_checkBox ->isChecked()){
      ros_pub->bfs_play_flag_ = true;
    }else{
      ros_pub->bfs_play_flag_ = false;
    }

}


void MainWindow::SetStamp(quint64 stamp)//根据时间戳调整滑块为主位置
{
  this->ui->time_stamp_label->setText(QString::number(stamp));
  if(slider_checker == false){
    ui->horizontalSlider->setValue(static_cast<int>(static_cast<float>(stamp - ros_pub->initial_data_stamp_)/static_cast<float>(10000000)));
  }
}

void MainWindow::play()//播放，把选择播放置灰，改变paly_flag_
{
    ui->imu_checkBox->setEnabled(false);
    ui->gnss_checkBox->setEnabled(false);
    ui->velodyne_16_checkBox->setEnabled(false);
    ui->velodyne_32_checkBox->setEnabled(false);
    ui->bfs_checkBox->setEnabled(false);
    ui->stereo_checkBox->setEnabled(false);

    if(ros_pub->play_flag_ == false){
      ros_pub->play_flag_ = true;
      this->ui->playButton->setText(QString::fromStdString("Pause"));
    }else{
      ros_pub->play_flag_ = false;
      this->ui->playButton->setText(QString::fromStdString("Play"));
    }
}



void MainWindow::slotDoubleSpinbox_slider()
{
    ui->pSlider->setValue((int)(ui->doubleSpinBox->value()*10));
}
void MainWindow::slotslider_DoubleSpinBox()
{
    ui->doubleSpinBox->setValue((double)(ui->pSlider->value())/10);
}
void MainWindow::PlaySpeedChange(double value)//倍速播放，传参给回调函数
{
    ros_pub->play_rate_ = value;
    speed_str = QString::number(value,'f',2);
}

void MainWindow::SliderPressed()
{
  slider_checker = true;
}


void MainWindow::SliderValueApply()
{
  ros_pub->ResetProcessStamp(slider_value);
  slider_checker = false;
}

void MainWindow::rangechange()//设置滑块范围
{
    this->ui->horizontalSlider->setRange(0,ros_pub->time_difference);
}




void MainWindow::SliderValueChange(int value)
{

   slider_value = value;
   if(loop_flag==true && value == ros_pub->time_difference-1){
       ros_pub->processed_stamp_ = 0;
       ros_pub->play_flag_ = true;
    }
}

void MainWindow::Restart()//重新开始
{
    if(ros_pub->play_flag_==true){
        ros_pub->play_flag_==false;
        ros_pub->processed_stamp_ = 0;
        this->ui->playButton->setText(QString::fromStdString("Pause"));
    }else{
        ros_pub->processed_stamp_ = 0;
    }


}

void MainWindow::Saveconfig()//保存配置文件
{

    //创建一个file文件
    QFileDialog fileDialog;
    QString fileName = fileDialog.getSaveFileName(this,tr("Open File"),"/config.txt",tr("Text File(*.txt)"));
    if(fileName == "")
    {
        return;
    }
    QFile file(fileName);//可以自己选择路径来保存文件名
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this,tr("ERROR"),tr("False to open "));
        return;
    }

    else
    {

        QTextStream textStream(&file);
        QString loop_str;
        QString velodyne_16_str;
        QString velodyne_32_str;
        QString stereo_str;
        QString bfs_str;
        QString gnss_str;
        QString imu_str;






        if(ui->loop ->isChecked()){
          loop_str = "true\n";
        }else{
          loop_str = "\n""false\n";
        }
        if(ui->velodyne_16_checkBox ->isChecked()){
          velodyne_16_str = "true\n";
        }else{
          velodyne_16_str = "false\n";
        }
        if(ui->velodyne_32_checkBox ->isChecked()){
          velodyne_32_str = "true\n";
        }else{
          velodyne_32_str = "false\n";
        }
        if(ui->stereo_checkBox ->isChecked()){
          stereo_str = "true\n";
        }else{
          stereo_str = "false\n";
        }
        if(ui->bfs_checkBox ->isChecked()){
            bfs_str = "true\n";
          }else{
            bfs_str = "false\n";
          }

        if(ui->gnss_checkBox ->isChecked()){
            gnss_str = "true\n";
          }else{
            gnss_str = "false\n";
          }
        if(ui->imu_checkBox ->isChecked()){
          imu_str = "true\n";
        }else{
          imu_str = "false\n";
        }

        textStream<<load_path + "\n";
        textStream<<speed_str;
        textStream<<loop_str;
        textStream<<velodyne_16_str;
        textStream<<velodyne_32_str;
        textStream<<stereo_str;
        textStream<<bfs_str;
        textStream<<gnss_str;
        textStream<<imu_str;

        QMessageBox::warning(this,tr("Hint"),tr("Successfully saved "));
        file.close();
    }
}
void MainWindow::Loadconfig()//加载配置文件
{
    //QStringList infoList;
    //获取文件名
    QString fileName = QFileDialog::getOpenFileName(
                this,
                tr("open a file."),
                "",
                tr("text(*.txt);;video files(*.avi *.mp4 *.wmv);;All files(*.*)"));
    QFile file(fileName);
    //打开文件
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug()<<"Can't open the file!"<<endl;
    }


        QString line1 = file.readLine();
        QString line2 = file.readLine();
        QString line3 = file.readLine();
        QString line4 = file.readLine();
        QString line5 = file.readLine();
        QString line6 = file.readLine();
        QString line7 = file.readLine();
        QString line8 = file.readLine();
        QString line9 = file.readLine();

        load_path = line1.left(line1.length() - 1);
        ros_pub->load_path_ = load_path.toUtf8().constData();
        double speed = line2.toDouble();
        this->ui->doubleSpinBox->setValue(speed);
        PlaySpeedChange(speed);
        if(line3 == "true\n"){
            loop_flag =true;
            ui->loop->setChecked(true);}
        else ui->loop->setChecked(false);
        if(line4 == "true\n") ui->velodyne_16_checkBox->setChecked(true);
        else ui->velodyne_16_checkBox->setChecked(false);
        if(line5 == "true\n") ui->velodyne_32_checkBox->setChecked(true);
        else ui->velodyne_32_checkBox->setChecked(false);
        if(line6 == "true\n") ui->stereo_checkBox->setChecked(true);
        else ui->stereo_checkBox->setChecked(false);
        if(line7 == "true\n") ui->bfs_checkBox->setChecked(true);
        else ui->bfs_checkBox->setChecked(false);
        if(line8 == "true\n") ui->gnss_checkBox->setChecked(true);
        else ui->gnss_checkBox->setChecked(false);
        if(line9 == "true\n") ui->imu_checkBox->setChecked(true);
        else ui->imu_checkBox->setChecked(false);

        Check_play_flag();
        ros_pub->Ready();
        this->ui->data_path->setText(load_path);

}

