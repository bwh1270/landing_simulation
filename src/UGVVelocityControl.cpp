#include <iostream>
#include <cmath>
#include <memory>
#include <cassert>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#define PI M_PI


class UGVVelocityControl
{
    private:
        // Output: Velocity Setpoint
        ros::Publisher setpointPub_;

        // Parameters for Step Input
        const float desiredMaxVel_;
        const float velStep_;
        float curVel_;

        // Parameters for Alternating Motion
        float vel_;
        const float dt_; 

    public:
        UGVVelocityControl(ros::NodeHandle *nh);
        ~UGVVelocityControl();

        void inputStep();

        void alternateMotion();

};

UGVVelocityControl::UGVVelocityControl(ros::NodeHandle *nh):
desiredMaxVel_(5.5f),
velStep_(0.5f),
dt_(1.0f)
{
    // Parameters for Step Input
    curVel_ = 0.0f;

    // Parameters for Alternating Motion
    vel_ = 1.0f;

    setpointPub_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("UGVVelocityControl class is initialized.");

    inputStep();
    // alternateMotion();
}

UGVVelocityControl::~UGVVelocityControl()
{
}

void UGVVelocityControl::inputStep()
{
    geometry_msgs::Twist velSpMsg;
    
    while (1) {
        assert( ("In this scenario, the velocity is always positive", curVel_ >= 0) );

        velSpMsg.linear.x = curVel_ + velStep_;
        setpointPub_.publish(velSpMsg);
        ros::Duration(1.5).sleep();
        curVel_ += velStep_;

        if (curVel_ >= desiredMaxVel_) {
            setpointPub_.publish(velSpMsg);
            ros::Duration(0.1).sleep();
            break;
        }
    }   
}

void UGVVelocityControl::alternateMotion()
{
    geometry_msgs::Twist velSpMsg;
    ros::Rate rate(20.0);
    ros::Time priorTime = ros::Time::now();

    velSpMsg.linear.x = vel_; // initial vel
    while (ros::ok()) {
        if (ros::Time::now().toSec() - priorTime.toSec() >= dt_) {
            vel_ = -vel_;
            priorTime = ros::Time::now();
        }

        velSpMsg.linear.x = vel_;
        setpointPub_.publish(velSpMsg);
        ros::spinOnce();
        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_vc_node");
    ros::NodeHandle nh("~");

    std::unique_ptr<UGVVelocityControl> ugv_velocity_control_obj = std::make_unique<UGVVelocityControl>(&nh);
    
    ros::spin();
    return 0;
}