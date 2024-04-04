#! /bin/bash

PREFIX="/home/hyunee/AIMS/PX4_PPNNPN/"
cd ${PREFIX}PX4-Autopilot

DONT_RUN=1 make px4_sitl gazebo-classic_typhoon_h480

source ${PREFIX}als_ws/devel/setup.bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

roslaunch landing_simulation sitl_als.launch
