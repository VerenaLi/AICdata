/**
 * @file /include/replay_tool/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef replay_tool_QNODE_HPP_
#define replay_tool_QNODE_HPP_
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QFileDialog>
#include <fstream>
#include <iostream>
#include <sstream>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QPixmap>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>
#include <algorithm>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <rosgraph_msgs/Clock.h>
#include <map>

#include <camera_info_manager/camera_info_manager.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <QStringListModel>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/locale.hpp>
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
#include <boost/tokenizer.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>








#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include<queue>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <std_msgs/Bool.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <time.h>

#include <mutex>
#include <ctime>
#include <thread>
#include<queue>
#include <condition_variable>




using namespace std;
using namespace cv;



//线程类
struct Data{

  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
  bool active_;

};







class Publishers : public QThread
{
    Q_OBJECT
public:
    explicit Publishers(QObject *parent = 0,QMutex *p_mutex = 0);

    ~Publishers();
    void ros_init(ros::NodeHandle &n);
    ros::NodeHandle nh_;
//    ros::NodeHandle left_camera_nh_;
//    ros::NodeHandle right_camera_nh_;
    bool init();
    void run();
    void Ready();
    QMutex *mutex_;

    void ResetProcessStamp(int position);//滑块相关


    int64_t begin_stamp;//滑块相关
    int64_t end_stamp;//滑块相关
    int time_difference;//滑块相关

//    boost::shared_ptr<camera_info_manager::CameraInfoManager> stereo_left_in;
//    boost::shared_ptr<camera_info_manager::CameraInfoManager> stereo_right_in;

//    boost::shared_ptr<camera_info_manager::CameraInfoManager> bfs_left_in;
//    boost::shared_ptr<camera_info_manager::CameraInfoManager> bfs_right_in;


















    string load_path_;//加载路径


    int64_t initial_data_stamp_;//stamp第一个时间戳
    int64_t last_data_stamp_;//stamp最后一个时间戳

    int stamp_show_count_;//无用

    bool play_flag_;//开始标志位
    bool pause_flag_;//暂停标志位
    bool loop_flag_;//无用
    double play_rate_;//倍速播放


    int imu_play_flag_;//checkbox确定的标志位，根据标志位判断是否加载数据和发布消息
    int velodyne_32_play_flag_;
    int velodyne_16_play_flag_;
    int gnss_play_flag_;
    int stereo_play_flag_;
    int bfs_play_flag_;



//在void Publishers::ros_init(ros::NodeHandle &n)
    ros::Publisher Velodyne_16_pub;
    ros::Publisher Velodyne_32_pub;
    ros::Publisher Stereo_left_pub;
    ros::Publisher Stereo_right_pub;
    ros::Publisher Bfs_left_pub;
    ros::Publisher Bfs_right_pub;
    ros::Publisher Gnss_pub;
    ros::Publisher Imu_pub;
    ros::Publisher Clock_pub;
    ros::Publisher Stereo_left_info_pub;//stereo/left/camera_info
    ros::Publisher Stereo_right_info_pub;//stereo/right/camera_info
    ros::Publisher Bfs_left_info_pub;
    ros::Publisher Bfs_right_info_pub;
    ros::Timer timer_;
    ros::Subscriber start_sub_;
    ros::Subscriber stop_sub_;


    void Velodyne_16_pub_thread();//发布消息
    void Velodyne_32_pub_thread();
    void Stereo_pub_thread();
    void Bfs_pub_thread();
    void Gnss_pub_thread();
    void Imu_pub_thread();
    void DataStampThread();

    vector<string> stereo_file_list_;





    cv::Mat stereo_left_image;
    cv::Mat stereo_right_image;

    cv::Mat bfs_left_image;
    cv::Mat bfs_right_image;


    void TimerCallback(const ros::TimerEvent&);
    int64_t processed_stamp_;//initial_data_stamp_+processed_stamp_ = 当前时间戳
    int64_t pre_timer_stamp_;
    bool reset_process_stamp_flag_;//滑块滑动等根据滑块位置重新设置时间戳




signals:
    void StampShow(quint64 stamp);//传递时间戳给界面函数
    void StartSignal();
    void changerange();//传递滑块范围给界面函数



Q_SIGNALS:
    void rosShutdown();


private:







        bool check_load_path(string path);//判断路径是否正确



        int get_velodyne_16_list(vector<string> &files);//写的一次性读全部bin文件名的函数，但是后来改了之后就用不到了
        int get_velodyne_32_list(vector<string> &files);
        vector<string> velodyne_16_file_list_;
        vector<string> velodyne_32_file_list_;



        int getTime_stamp();//读time_stamp文件
        int get_velodyne_16(int64_t stamp);//按照时间戳从数据集读数据，激光雷达和相机一次性读一个，stamp、imu和gnss开始时在Ready函数全部
        int get_velodyne_32(int64_t stamp);
        int get_stereo(int64_t stamp);
        int get_bfs(int64_t stamp);
        int getGNSS();
        int getIMU();



    int64_t prev_clock_stamp_;






    sensor_msgs::PointCloud2 publish_16_cloud;//
    sensor_msgs::PointCloud2 publish_32_cloud;
    pcl::PointCloud<pcl::PointXYZI> velodyne_16_cloud;
    pcl::PointCloud<pcl::PointXYZI> velodyne_32_cloud;





    pair<string,cv::Mat> stereo_left_next_img_;//
    pair<string,cv::Mat> stereo_right_next_img_;



    pair<string,cv::Mat> bfs_left_next_img_;
    pair<string,cv::Mat> bfs_right_next_img_;








    multimap<int64_t, string>               data_stamp_;  //map只允许key与 value一一对应；multimap一个key可对应多个value，存放stamp的容器

    map<int64_t, sensor_msgs::NavSatFix>    gnss_data_;//存放gnss数据的容器


    map<int64_t, sensor_msgs::Imu>         imu_data_;



    Data data_stamp_thread_;//定义的多线程
    Data gnss_thread_;
    Data imu_thread_;
    Data bfs_thread_;
    Data stereo_thread_;
    Data velodyne_16_thread_;
    Data velodyne_32_thread_;

    map<int64_t, int64_t> stop_period_; //start and stop stamp


    bool velodyne_16;//是否读取完数据的标志位
    bool velodyne_32;
    bool imu;
    bool gnss;
    bool stereo;
    bool bfs;
    int64_t imu_stamp;//需要赋值给各个线程时间戳
    int64_t gnss_stamp;
    int64_t velodyne_16_stamp;
    int64_t velodyne_32_stamp;
    int64_t stereo_stamp;
    int64_t bfs_stamp;







public slots:

};// namespace replay_tool

#endif /* replay_tool_QNODE_HPP_ */
