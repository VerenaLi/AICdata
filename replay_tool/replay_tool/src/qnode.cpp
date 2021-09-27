
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/replay_tool/qnode.hpp"
#include <QApplication>
#include <QFileDialog>
#include <QDir>
#include <QString>
#include <QStringList>
#include <QDebug>




#include <asm/ioctl.h>
#include <asm/ioctls.h>



#include <QMutexLocker>
using namespace std;


Publishers::Publishers(QObject *parent,QMutex *p_mutex) :
    QThread(parent), mutex_(p_mutex)
{
    processed_stamp_ = 0;
    play_rate_ = 1.0;
    loop_flag_ = false;


    reset_process_stamp_flag_ = true;
    prev_clock_stamp_ = 0;


    imu_play_flag_ = false;
    gnss_play_flag_ = false;
    stereo_play_flag_ = false;
    bfs_play_flag_ = false;

    velodyne_32_play_flag_ = false;
    velodyne_16_play_flag_ = false;



    velodyne_16 = false;
    velodyne_32 = false;
    imu = false;
    gnss = false;
    stereo = false;
    bfs = false;


}





Publishers::~Publishers() {
    data_stamp_thread_.active_ = false;
    gnss_thread_.active_ = false;
    imu_thread_.active_ = false;
    velodyne_16_thread_.active_ = false;
    velodyne_32_thread_.active_ = false;
    stereo_thread_.active_ = false;
    bfs_thread_.active_ = false;

    usleep(100000);

    data_stamp_thread_.cv_.notify_all();
    if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

    gnss_thread_.cv_.notify_all();
    if(gnss_thread_.thread_.joinable()) gnss_thread_.thread_.join();

    imu_thread_.cv_.notify_all();
    if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();

    velodyne_16_thread_.cv_.notify_all();
    if(velodyne_16_thread_.thread_.joinable()) velodyne_16_thread_.thread_.join();

    velodyne_32_thread_.cv_.notify_all();
    if(velodyne_32_thread_.thread_.joinable()) velodyne_32_thread_.thread_.join();

    stereo_thread_.cv_.notify_all();
    if(stereo_thread_.thread_.joinable()) stereo_thread_.thread_.join();

    bfs_thread_.cv_.notify_all();
    if(bfs_thread_.thread_.joinable()) bfs_thread_.thread_.join();
}


void Publishers::ros_init(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  //万分之一秒执行一次，不是只执行一次

  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&Publishers::TimerCallback, this, _1));//ros定时器，定时回调函数
  Gnss_pub = nh_.advertise<sensor_msgs::NavSatFix>("gnss", 1000);
  Imu_pub = nh_.advertise<sensor_msgs::Imu>("imu_pub", 1000);
  Velodyne_32_pub = nh_.advertise<sensor_msgs::PointCloud2>("velodyne_32", 1000);
  Velodyne_16_pub = nh_.advertise<sensor_msgs::PointCloud2>("velodyne_16", 1000);
  Bfs_left_pub = nh_.advertise<sensor_msgs::Image>("bfs_left_pub", 10);
  Bfs_right_pub = nh_.advertise<sensor_msgs::Image>("bfs_right_pub", 10);
  Stereo_left_pub = nh_.advertise<sensor_msgs::Image>("stereo_left_pub", 10);
  Stereo_right_pub = nh_.advertise<sensor_msgs::Image>("stereo_right_pub", 10);
  Stereo_left_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("stereo_left", 10);
  Stereo_right_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("stereo_right", 10);
  Bfs_left_pub = nh_.advertise<sensor_msgs::Image>("bfs_left_pub", 10);
  Bfs_right_pub = nh_.advertise<sensor_msgs::Image>("bfs_right_pub", 10);
  Bfs_left_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("bfs_left", 10);
  Bfs_right_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("bfs_right", 10);
//  left_camera_nh_ = ros::NodeHandle(nh_,"left");
//  right_camera_nh_ = ros::NodeHandle(nh_,"right");
//  stereo_left_in = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(left_camera_nh_,"/stereo/left"));
//  stereo_right_in = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(right_camera_nh_,"/stereo/right"));
  Clock_pub = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

}







void Publishers::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}



void Publishers::Ready()
{
    data_stamp_thread_.active_ = false;
    data_stamp_thread_.cv_.notify_all();
    if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();
    gnss_thread_.active_ = false;
    gnss_thread_.cv_.notify_all();
    if(gnss_thread_.thread_.joinable()) gnss_thread_.thread_.join();
    imu_thread_.active_ = false;
    imu_thread_.cv_.notify_all();
    if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();
    velodyne_16_thread_.active_ = false;
    velodyne_16_thread_.cv_.notify_all();
    if(velodyne_16_thread_.thread_.joinable()) velodyne_16_thread_.thread_.join();
    velodyne_32_thread_.active_ = false;
    velodyne_32_thread_.cv_.notify_all();
    if(velodyne_32_thread_.thread_.joinable()) velodyne_32_thread_.thread_.join();
    stereo_thread_.active_ = false;
    stereo_thread_.cv_.notify_all();
    if(stereo_thread_.thread_.joinable()) stereo_thread_.thread_.join();
    bfs_thread_.active_ = false;
    bfs_thread_.cv_.notify_all();
    if(bfs_thread_.thread_.joinable()) bfs_thread_.thread_.join();



//检查路径是否正确
    if(check_load_path(load_path_+"/sensor_data/")==true){
           cout << "Successfully loaded sensor data" << endl;
           getTime_stamp();
       }else{
           cout << "Please check the input path. Sensor data does not exist " << endl;
           return;
       }



    if(imu_play_flag_ == true){
        if(check_load_path(load_path_+"/sensor_data/xsens_imu.csv")==true){
            getIMU();//读imu数据
        }else{
            imu_play_flag_ = false;
                  cout << "Please check the dataset. Imu data does not exist " << endl;
           }
       }


    if(gnss_play_flag_ == true){
        if(check_load_path(load_path_+"/sensor_data/gps.csv")==true){
            getGNSS();//读gnss数据
        }else{
            gnss_play_flag_ = false;
            cout << "Please check the dataset. Gnss data does not exist " << endl;
        }
    }


    if(velodyne_16_play_flag_ == true){
        if(check_load_path(load_path_+"/sensor_data/VLP_left/")==true){
            cout << "Successfully loaded Velodyne_16 data" << endl;
        }else{
            velodyne_16_play_flag_ = false;
            cout << "Please check the dataset. Velodyne_16 data does not exist " << endl;

        }
    }


    if(velodyne_32_play_flag_ == true){
        if(check_load_path(load_path_+"/sensor_data/VLP_right/")==true){
            cout << "Successfully loaded Velodyne_32 data" << endl;
        }else{
            velodyne_32_play_flag_ = false;
        cout << "Please check the dataset. Velodyne_32 data does not exist " << endl;}
    }

    if(stereo_play_flag_ == true){
        if(check_load_path(load_path_+"/image/stereo_left/")==true && check_load_path(load_path_+"/image/stereo_right/")==true){
            cout << "Successfully loaded Stereo data" << endl;
        }else{
            stereo_play_flag_ = false;
        cout << "Please check the dataset. Stereo data does not exist " << endl;}
    }

    if(bfs_play_flag_ == true){
        if(check_load_path(load_path_+"/image/bfs_left/")==true && check_load_path(load_path_+"/image/bfs_right/")==true){
            cout << "Successfully loaded Bfs data" << endl;
        }else{
            bfs_play_flag_ = false;
        cout << "Please check the dataset. Bfs data does not exist " << endl;}
    }




    data_stamp_thread_.active_ = true;
    gnss_thread_.active_ = true;
    imu_thread_.active_ = true;
    velodyne_16_thread_.active_ = true;
    velodyne_32_thread_.active_ = true;
    stereo_thread_.active_ = true;
    bfs_thread_.active_ = true;


    data_stamp_thread_.thread_ = std::thread(&Publishers::DataStampThread,this);
    gnss_thread_.thread_ = std::thread(&Publishers::Gnss_pub_thread,this);
    imu_thread_.thread_ = std::thread(&Publishers::Imu_pub_thread,this);
    velodyne_16_thread_.thread_ = std::thread(&Publishers::Velodyne_16_pub_thread,this);
    velodyne_32_thread_.thread_ = std::thread(&Publishers::Velodyne_32_pub_thread,this);
    bfs_thread_.thread_ = std::thread(&Publishers::Bfs_pub_thread,this);
    stereo_thread_.thread_ = std::thread(&Publishers::Stereo_pub_thread,this);

}



//根据时间戳对比需要唤醒的线程
void Publishers::DataStampThread(){

    for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){

      auto stamp = iter->first;

      while((stamp >= (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
        if(processed_stamp_ == 0){
            iter = data_stamp_.begin();
            stamp = iter->first;
        }
        usleep(1);
        if(reset_process_stamp_flag_ == true) break;
      }



//
      if(reset_process_stamp_flag_ == true){
          int64_t target_stamp = processed_stamp_ + initial_data_stamp_;
          iter = data_stamp_.lower_bound(target_stamp);//lower_bound功能：查找非递减序列[first,last) 内第一个大于或等于某个元素的位置。
          reset_process_stamp_flag_ = false;
          continue;
        }


      if(data_stamp_thread_.active_ == false) return;
      if(iter->second.compare("gps") == 0 && gnss_play_flag_ ==  true){
          gnss_stamp = stamp ;
          gnss = true;
        gnss_thread_.cv_.notify_all();
      }else if(iter->second.compare("imu") == 0 && imu_play_flag_ ==  true){
        imu_stamp = stamp ;
        imu = true ;
        imu_thread_.cv_.notify_all();
      }else if(iter->second.compare("velodyne_left") == 0 && velodyne_16_play_flag_ == true){
          get_velodyne_16(stamp);
          velodyne_16_thread_.cv_.notify_all();
      }else if(iter->second.compare("velodyne_right") == 0 && velodyne_32_play_flag_ == true){
          get_velodyne_32(stamp);
          velodyne_32_thread_.cv_.notify_all();
     }else if(iter->second.compare("stereo") == 0 && stereo_play_flag_ ==  true){
          get_stereo(stamp);
          stereo_thread_.cv_.notify_all();
      }else if(iter->second.compare("bfs") == 0 && bfs_play_flag_ ==  true){
          get_bfs(stamp);
          bfs_thread_.cv_.notify_all();
      }

      emit StampShow(stamp);//传时间戳到界面函数


      //发布时间消息
      if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
          rosgraph_msgs::Clock clock;
          clock.clock.fromNSec(stamp);
          Clock_pub.publish(clock);
          prev_clock_stamp_ = stamp;
      }
    }
    std::cout << "Data publish complete" << endl;
}







//回调函数，受定时器控制
void Publishers::TimerCallback(const ros::TimerEvent&){


//    ros::Rate loop_rate();

//    loop_rate.sleep();
    int64_t current_stamp = ros::Time::now().toNSec();
    if(play_flag_ == true)
        processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    pre_timer_stamp_ = current_stamp;
    //cout<<last_expected<<endl;
}



//检查加载路径函数

bool Publishers::check_load_path(string path)
{
    ifstream f(path.c_str());
    if(!f.good()){
       //cout << "Please check the input path."<<"'"<< path <<"'"<< " does not exist " << endl;
       return false;
    }else
        return true;
}

//int fileNameFilter(const struct dirent *cur) {
//    string str(cur->d_name);
//    if (str.find(".bin") != string::npos) {
//        return 1;
//    }
//    return 0;
//}

//int Publishers::get_velodyne_16_list(vector<string> &files)
//{

//    string filename;
//    filename = load_path_+"/sensor_data/VLP_left";
//    vector<string> ret;
//    struct dirent **namelist;
//    int n;

////    //scandir函数：读取特定的目录
////    // scandir()会扫描参数dir指定的目录文件，经由参数select指定的函数来挑选目录结构至参数namelist数组中，最后再调用

//    if ((n= scandir(filename.c_str(), &namelist, fileNameFilter, alphasort)) < 0)
//        return 0;
//    for (int i = 0; i < n; ++i) {
//        string filePath(namelist[i]->d_name);
//        ret.push_back(filePath);
//        free(namelist[i]);
//    };
//    free(namelist);
//    for(auto iter = ret.rbegin() ; iter!= ret.rend() ; iter++){
//      files.push_back(*iter);
//    }
//      return 0;





//}

//int Publishers::get_velodyne_32_list(vector<string> &files)
//{
//    string filename;
//    filename = load_path_+"/sensor_data/VLP_right";
//    vector<string> ret;
//    struct dirent **namelist;
//    int n;
////    //scandir函数：读取特定的目录数据
////    // scandir()会扫描参数dir指定的目录文件，经由参数select指定的函数来挑选目录结构至参数namelist数组中，最后再调用

//    if ((n= scandir(filename.c_str(), &namelist, fileNameFilter, alphasort)) < 0)
//        return 0;
//    for (int i = 0; i < n; ++i) {
//        string filePath(namelist[i]->d_name);
//        ret.push_back(filePath);
//        free(namelist[i]);
//    };
//    free(namelist);
//    for(auto iter = ret.rbegin() ; iter!= ret.rend() ; iter++){
//      files.push_back(*iter);
//    }
//      return 0;





//}




//一次性读时间戳文件的到容器中
int Publishers::getTime_stamp()
{
    FILE *fp;
    int64_t stamp;
    string filename;
    filename = load_path_+"/sensor_data/data_stamp.csv" ;
    char data_name[50];
    fp = fopen(filename.c_str(),"r");

    while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
      data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
    }

    cout << "Successfully loaded Stamp data" << endl;
    fclose(fp);

    initial_data_stamp_ = data_stamp_.begin()->first ;
    last_data_stamp_ = prev(data_stamp_.end(),1)->first ;
    begin_stamp = initial_data_stamp_/10000000;
    end_stamp = last_data_stamp_/10000000;
    time_difference = end_stamp -begin_stamp;

    emit changerange();//传参到界面函数
}



//一次性读取gnss到map容器
int Publishers::getGNSS(){

    FILE *fp;
    int64_t stamp;
    string filename;


    filename =load_path_+"/sensor_data/gps.csv";

    fp = fopen(filename.c_str(),"r");
    double array[13];
    sensor_msgs::NavSatFix gps_data;
    gnss_data_.clear();


    while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&array[0],&array[1],&array[2],&array[3],&array[4],&array[5],&array[6],&array[7],&array[8],&array[9],&array[10],&array[11]) == 13){
      gps_data.header.stamp.fromNSec(stamp);
      gps_data.header.frame_id = "gnss";
      gps_data.latitude = array[0];
      gps_data.longitude = array[1];
      gps_data.altitude = array[2];
      gps_data.position_covariance[0] = array[3];
      gps_data.position_covariance[1] = array[4];
      gps_data.position_covariance[2] = array[5];
      gps_data.position_covariance[3] = array[6];
      gps_data.position_covariance[4] = array[7];
      gps_data.position_covariance[5] = array[8];
      gps_data.position_covariance[6] = array[9];
      gps_data.position_covariance[7] = array[10];
      gps_data.position_covariance[8] = array[11];
      gnss_data_[stamp] = gps_data;
    }
    cout << "Successfully loaded Gnss data" << endl;
    fclose(fp);


}


//读imu数据，一次性读取整个.csv文件到map容器
int Publishers::getIMU()
{

        FILE *fp;
        int64_t stamp;
        string filename;
        filename =load_path_+"/sensor_data/xsens_imu.csv";
        fp = fopen(filename.c_str(),"r");

         sensor_msgs::Imu imu_data;
          imu_data_.clear();
                    double array[16];


                    while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&array[0],&array[1],&array[2],&array[3],&array[4],&array[5],&array[6],&array[7],&array[8],&array[9],&array[10],&array[11],&array[12],&array[13],&array[14],&array[15]) ==17)
                    {

                        imu_data.header.stamp.fromNSec(stamp);
                        imu_data.header.frame_id = "imu";
                        imu_data.orientation.x = array[0];
                        imu_data.orientation.y = array[1];
                        imu_data.orientation.z = array[2];
                        imu_data.orientation.w = array[3];
                        imu_data.angular_velocity.x = array[7];
                        imu_data.angular_velocity.y = array[8];
                        imu_data.angular_velocity.z = array[9];
                        imu_data.linear_acceleration.x = array[10];
                        imu_data.linear_acceleration.y = array[11];
                        imu_data.linear_acceleration.z = array[12];



                        imu_data.orientation_covariance[0] = 3;
                        imu_data.orientation_covariance[4] = 3;
                        imu_data.orientation_covariance[8] = 3;
                        imu_data.angular_velocity_covariance[0] = 3;
                        imu_data.angular_velocity_covariance[4] = 3;
                        imu_data.angular_velocity_covariance[8] = 3;
                        imu_data.linear_acceleration_covariance[0] = 3;
                        imu_data.linear_acceleration_covariance[4] = 3;
                        imu_data.linear_acceleration_covariance[8] = 3;


                        imu_data_[stamp] = imu_data;

                   }
            cout << "Successfully loaded Imu data" << endl;
            fclose(fp);



}





//读velodyne_16数据，每次只读一个.bin文件
int Publishers::get_velodyne_16(int64_t stamp){

        ifstream file;
        string filename = load_path_ + "/sensor_data/VLP_left" +"/"+ to_string(stamp) + ".bin";
        pcl::PointXYZI point;
        file.open(filename, ios::in|ios::binary);


        while(!file.eof()){

            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
            velodyne_16_cloud.points.push_back (point);

}

        file.close();
        pcl::toROSMsg(velodyne_16_cloud, publish_16_cloud); //pcl::PointCloud <pcl::PointXYZ>转pcl::PCLPointCloud2   pcl::fromPCLPointCloud2(pcl::PointCloud<pcl::PointXYZ>,pcl::PCLPointCloud2);
        velodyne_16_cloud.clear();
        velodyne_16_stamp = stamp;
        velodyne_16 = true;
        return 0;


}


//读velodyne_32数据，每次只读一个.bin文件

int Publishers::get_velodyne_32(int64_t stamp)
{

    ifstream file;

    string filename = load_path_ + "/sensor_data/VLP_right" +"/"+ to_string(stamp) + ".bin";

    pcl::PointXYZI point;
    file.open(filename, ios::binary);

while(!file.eof()){


file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
velodyne_32_cloud.points.push_back (point);

}
    file.close();
    pcl::toROSMsg(velodyne_32_cloud, publish_32_cloud); //pcl::PointCloud <pcl::PointXYZ>转pcl::PCLPointCloud2//pcl::fromPCLPointCloud2(pcl::PointCloud<pcl::PointXYZ>,pcl::PCLPointCloud2);
    velodyne_32_cloud.clear();
    velodyne_32_stamp = stamp;
    velodyne_32 = true;
    return 0;
}


//读取stereo数据，每次读取左右各一张png

int Publishers::get_stereo(int64_t stamp)
{

    string left_path;
    left_path = load_path_ + "/image/stereo_left" +"/"+ to_string(stamp)+".png";
    string right_path;
    right_path = load_path_ + "/image/stereo_right" +"/"+ to_string(stamp)+".png";
    stereo_left_image = imread(left_path, CV_LOAD_IMAGE_ANYDEPTH);
    stereo_right_image = imread(right_path, CV_LOAD_IMAGE_ANYDEPTH);
    stereo_stamp = stamp;
    stereo = true;
    return 0;


}


//读取bfs数据，每次读取左右各一张png
int Publishers::get_bfs(int64_t stamp)
{

    string left_path;
    left_path = load_path_ + "/image/bfs_left" +"/"+ to_string(stamp)+".png";
    string right_path;
    right_path = load_path_ + "/image/bfs_right" +"/"+ to_string(stamp)+".png";
    bfs_left_image = imread(left_path, CV_LOAD_IMAGE_ANYDEPTH);
    bfs_right_image = imread(right_path, CV_LOAD_IMAGE_ANYDEPTH);
    bfs_stamp = stamp;
    bfs = true;
    return 0;


}





//发布imu消息
void Publishers::Imu_pub_thread(){

    while(1){
      std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
      imu_thread_.cv_.wait(ul);
      if(imu_thread_.active_ == false) return;
      ul.unlock();

      while(imu == true){
          Imu_pub.publish(imu_data_[imu_stamp]);
          imu = false;
      }
      if(imu_thread_.active_ == false) return;
    }
}


//发布gnss消息
void Publishers::Gnss_pub_thread()
{
    while(1){
      std::unique_lock<std::mutex> ul(gnss_thread_.mutex_);
      gnss_thread_.cv_.wait(ul);
      if(gnss_thread_.active_ == false) return;
      ul.unlock();
      while(gnss == true){

          Gnss_pub.publish(gnss_data_[gnss_stamp]);

          gnss = false;
      }
      if(gnss_thread_.active_ == false) return;
    }
}

//发布velodyne_16消息
void Publishers::Velodyne_16_pub_thread()
{
    while(1){
      std::unique_lock<std::mutex> ul(velodyne_16_thread_.mutex_);
      velodyne_16_thread_.cv_.wait(ul);
      if(velodyne_16_thread_.active_ == false) return;
      ul.unlock();
      while(velodyne_16 == true){
              publish_16_cloud.header.stamp.fromNSec(velodyne_16_stamp);
              publish_16_cloud.header.frame_id = "velodyne_16";
              Velodyne_16_pub.publish(publish_16_cloud);
              velodyne_16_cloud.clear();
              velodyne_16 = false ;
      }

      if(velodyne_16_thread_.active_ == false) return;
    }


}


//发布velodyne_32消息
void Publishers:: Velodyne_32_pub_thread()
{
    while(1){
      std::unique_lock<std::mutex> ul(velodyne_32_thread_.mutex_);
      velodyne_32_thread_.cv_.wait(ul);
      if(velodyne_32_thread_.active_ == false) return;
      ul.unlock();
      while(velodyne_32 == true){

              publish_32_cloud.header.stamp.fromNSec(velodyne_32_stamp);
              publish_32_cloud.header.frame_id = "left_velodyne";
              Velodyne_32_pub.publish(publish_32_cloud);
              velodyne_32_cloud.clear();
              velodyne_32 = false;


  }

      if(velodyne_32_thread_.active_ == false) return;
    }

}

//发布stereo消息

void Publishers::Stereo_pub_thread()
{
    while(1){
      std::unique_lock<std::mutex> ul(stereo_thread_.mutex_);
      stereo_thread_.cv_.wait(ul);
      if(stereo_thread_.active_ == false) return;
      ul.unlock();

      while(stereo == true){

          cv_bridge::CvImage left_out_msg;
          left_out_msg.header.stamp.fromNSec(stereo_stamp);
          left_out_msg.header.frame_id = "stereo_left";
          left_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          left_out_msg.image    = stereo_left_image;

          cv_bridge::CvImage right_out_msg;
          right_out_msg.header.stamp.fromNSec(stereo_stamp);
          right_out_msg.header.frame_id = "stereo_right";
          right_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          right_out_msg.image    = stereo_right_image;

          sensor_msgs::CameraInfo stereo_left_info_;
          sensor_msgs::CameraInfo stereo_right_info_;
          stereo_left_info_.header.stamp.fromNSec(stereo_stamp);
          stereo_left_info_.header.frame_id = "stereo/left";
          stereo_right_info_.header.stamp.fromNSec(stereo_stamp);
          stereo_right_info_.header.frame_id = "stereo/right";

          Stereo_left_pub.publish(left_out_msg.toImageMsg());
          Stereo_right_pub.publish(right_out_msg.toImageMsg());

          Stereo_left_info_pub.publish(stereo_left_info_);
          Stereo_right_info_pub.publish(stereo_right_info_);
          stereo = false;
      }
      if(stereo_thread_.active_ == false) return;
    }
}

//发布bfs消息
void Publishers::Bfs_pub_thread()
{
    while(1){
      std::unique_lock<std::mutex> ul(bfs_thread_.mutex_);
      bfs_thread_.cv_.wait(ul);
      if(bfs_thread_.active_ == false) return;
      ul.unlock();

      while(bfs == true){

          cv_bridge::CvImage left_out_msg;
          left_out_msg.header.stamp.fromNSec(bfs_stamp);
          left_out_msg.header.frame_id = "bfs_left";
          left_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          left_out_msg.image    = bfs_left_image;

          cv_bridge::CvImage right_out_msg;
          right_out_msg.header.stamp.fromNSec(bfs_stamp);
          right_out_msg.header.frame_id = "bfs_right";
          right_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          right_out_msg.image    = bfs_right_image;

          sensor_msgs::CameraInfo bfs_left_info_;
          sensor_msgs::CameraInfo bfs_right_info_;
          bfs_left_info_.header.stamp.fromNSec(bfs_stamp);
          bfs_left_info_.header.frame_id = "bfs/left";
          bfs_right_info_.header.stamp.fromNSec(bfs_stamp);
          bfs_right_info_.header.frame_id = "bfs/right";

          Bfs_left_pub.publish(left_out_msg.toImageMsg());
          Bfs_right_pub.publish(right_out_msg.toImageMsg());

          Bfs_left_info_pub.publish(bfs_left_info_);
          Bfs_right_info_pub.publish(bfs_right_info_);
          bfs = false;
      }
      if(bfs_thread_.active_ == false) return;
    }
}



//移动滑块后，滑块根据滑块位置设置processed_stamp_

void Publishers::ResetProcessStamp(int position)
{
  if(position > 0 && position < time_difference){
      processed_stamp_ = static_cast<float>(position)*10000000;
    reset_process_stamp_flag_ = true;
  }

}



