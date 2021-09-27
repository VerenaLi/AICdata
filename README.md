# AICdata


This program is a replay tool for our dataset. 



## Prerequisites

We have tested the library in **Ubuntu 18.04**,  but it should be easy to compile in other platforms. 

### ROS 

We use  [ROS](https://www.ros.org/) thread to publish the data as ROS messages of a real-time process. You can install it easily by refer this [website](http://wiki.ros.org/melodic/Installation/Ubuntu).

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-desktop-full

sudo apt-get install python-rosdep

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### QT  pkg support

Our GUI is created using ROS-QT, so you need to install qt-pkg support.

```shell

sudo apt-get install ros-melodic-qt-create
sudo apt-get install ros-melodic-qt-build
```



## Building and running

```shell
#build workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://....git replay_tool
cd ..
catkin_make

#run repalt tool
source devel/setup.bash
roslaunch replay_tool replay_tool.launch 
```



## Load and play the data 

1. Click 'Load' button and choose data set folder including sensor_data folder then click 'Open'.
2. The 'Play' button starts publishing data as ROS messages.
3. The 'Pause' button stop publishing data.
4. The 'Restart' button allows you to play data from the beginning.
5. .You can set the playback speed you want.
6. In the 'Publisher choice' column you can select the type of sensor data you need to play.
7. You can use 'Settings - Save config' to save this setting as a config file.
8. Load your setting by 'Settings - Load config'.
