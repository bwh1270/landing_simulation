#include <iostream>
#include <cassert>
#include <string>
#include <memory>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
using namespace std;


class SetpointPublisher
{
    private:
        ros::Publisher setpointPub_;
        ros::Subscriber stateSub_;
        ros::ServiceClient setModeClient_;

        std::string flightMode_;
        bool IsOffboard_;

        void stateCb(const mavros_msgs::State::ConstPtr& msg);
        void setOffboardMode();

        void stepInput();
        // void rampInput();
        // void parabolicInput();
        void inputStepAndRamp();

    public:
        SetpointPublisher(ros::NodeHandle* nh);
        ~SetpointPublisher();


};

SetpointPublisher::SetpointPublisher(ros::NodeHandle* nh)
{
    setpointPub_ = nh->advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 5);
    stateSub_  = nh->subscribe("/mavros/state", 10, &SetpointPublisher::stateCb, this);
    setModeClient_ = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    IsOffboard_ = false;

    // stepInput();
    // rampInput();
    inputStepAndRamp();

}

SetpointPublisher::~SetpointPublisher()
{
}

void SetpointPublisher::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
    flightMode_ = msg->mode;
}

void SetpointPublisher::setOffboardMode()
{
    /* This function cannot change mode to OFFBOARD alone.
	 * If you want to change offboard mode then,
	 * publishing to setpoint topics
	 */
	std::string mode = "OFFBOARD";
	ROS_INFO("Setting mode to %s", mode.c_str());

	mavros_msgs::SetMode modeSrv;
	modeSrv.request.custom_mode = mode;

	if (setModeClient_.call(modeSrv) && modeSrv.response.mode_sent) {
		ROS_INFO("Mode switch request is sent successfully");
	}
	else {
		ROS_WARN("Mode switch request is not sent");
		return;
	}
}

void SetpointPublisher::stepInput()
{
    double step[2] = {0.0, 5.5556};
    float interval = 6.f;
    float endTime = 30.f;
    int idx = 0;

    geometry_msgs::Twist velSp;
    velSp.linear.x = step[0];
    // velSp.linear.y = step[0];
    // velSp.linear.z = step[0];

    ros::Rate rate(100.0);

    while (ros::ok()) {
        if (flightMode_ == "OFFBOARD") {
            break;
        }
        setpointPub_.publish(velSp);
        ros::spinOnce();
    }

    ROS_INFO("Step Input signal is published..!");
    ros::Time beginTime = ros::Time::now();
    ros::Time priorTime = ros::Time::now();
    while (flightMode_ == "OFFBOARD") {
        if (ros::Time::now().toSec() - beginTime.toSec() >= endTime) {
            ROS_INFO("Step Input signal is terminated..!");
            break;
        }

        if (ros::Time::now().toSec() - priorTime.toSec() >= interval) {
            idx++;
            velSp.linear.x = step[idx%2];
            // velSp.linear.y = step[0];
            // velSp.linear.z = step[0];
            priorTime = ros::Time::now();
            ROS_INFO("step");
        
        }

        setpointPub_.publish(velSp);
        ros::spinOnce();
        rate.sleep();
    }
    return;
}

void SetpointPublisher::inputStepAndRamp()
{
    double step[4] = {0.0, 1.0, 3.0, 5.55556};
    double ramp[3] = {1.0, 1.5, 2.0};
    float interval = 10.f;
    int idx = 0;
    int lastIdx = 2*((sizeof(step)/sizeof(step[0])-1) + (sizeof(ramp)/sizeof(ramp[0])));

    ros::Time beginTime;
    ros::Time priorTime;
    // double duration = interval*100;
    geometry_msgs::Twist velSp;

    float f = 50.0;
    float dt = 1./f;
    ros::Rate rate(f);

    while (ros::ok())
    {
        if (IsOffboard_) 
        {
            if (ros::Time::now().toSec() - priorTime.toSec() >= interval) {
                idx++;
                priorTime = ros::Time::now();

                if (idx > lastIdx) {
                    ROS_INFO("Reference Input is terminated..!");
                    break;
                }

                ROS_INFO("reference input is changed");
            }

            if (idx%2 == 0) {
                velSp.linear.x = step[0];

            } else if (idx < 7) {
                velSp.linear.x = step[idx%2+idx/2];

            } else {
                velSp.linear.x += ramp[(idx-7)/2]*dt;
                if (velSp.linear.x >= 12) {
                    velSp.linear.x = 12.0;
                }
                
                if (idx == lastIdx) {

                }
            }

            setpointPub_.publish(velSp);
            ros::spinOnce();
            rate.sleep();

        } else 
        {
            if (flightMode_ == "OFFBOARD") {
                IsOffboard_ = true;
                beginTime = ros::Time::now();
                priorTime = ros::Time::now();
                ROS_INFO("Offboard mode is detected..!");
                ROS_INFO("Reference Signal is processed..!");
                continue;
            }

            velSp.linear.x = 0.0;
            velSp.linear.y = 0.0;
            velSp.linear.z = 0.0;
            setpointPub_.publish(velSp);
            ros::spinOnce();
            rate.sleep();
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "setpoint_publisher_node");
    ros::NodeHandle nh("~");

    std::unique_ptr<SetpointPublisher> setpoint_publisher_obj = std::make_unique<SetpointPublisher>(&nh);

    ros::spin();
    return 0;
}