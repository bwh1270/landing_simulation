#ifndef __TF_PUBLISHER__
#define __TF_PUBLISHER__

#include <cstdio>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"


class TfTalker
{
    private:
        ros::Subscriber uav_pose_sub_;
        std::string uav_frame_;
        std::string cam_frame_;

    public:
        TfTalker(ros::NodeHandle *nh);
        ~TfTalker();
        void tfCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif