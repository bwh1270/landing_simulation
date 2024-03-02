#ifndef __HIGH_LEVEL_CONTROL_CLASS__
#define __HIGH_LEVEL_CONTROL_CLASS__

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>


class FlightMode
{
	private:
		ros::Subscriber current_state_sub_;
		ros::ServiceClient set_mode_client_;
		ros::ServiceClient arm_client_;
		ros::ServiceClient takeoff_client_;
		ros::ServiceClient land_client_;
		
		mavros_msgs::State state_;  // current flight state
		float freq_;
		
	public:
		FlightMode(ros::NodeHandle *nh);
        ~FlightMode();

		void currentStateCallback(const mavros_msgs::State::ConstPtr &msg);
		void setConnection();
		void setArm();
		void setOffboardMode();
		void land();
};


struct pose
{
    ros::Time time_stamp;
    Eigen::Vector3d pos;
    Eigen::Vector4d q;
};

struct setpoint
{
    ros::Time time_stamp;
    Eigen::Vector3d vec;
};


class HighLevelControl
{
    private:
        FlightMode fmode_obj_;
        bool is_offboard_; // check whether the flight mode is OFFBOARD 
        bool is_estimated_data_; // check whether the estimated data is subscribed
        std::string uav_frame_id_;
        bool debug_;

        // Controller
        pose uav_current_pose_;
        setpoint pos_setpoint_; // time stamp and desired x,y,z
        setpoint vel_setpoint_; // time_stamp and output of position controller
        setpoint ff_velocity_; // feedforward velocity term which is estimated by kalman filter
        Eigen::Vector4d pidff_gains_; // PID and Feedforward Gains
        Eigen::Vector3d e_;
        Eigen::Vector3d e_old_;
        double rate_;
        bool first_pass_; // because first loop has error_old to zero.
        
        ros::Subscriber start_sub_;
        ros::Subscriber current_pose_sub_;
        ros::Subscriber estimate_sub_;
        ros::Subscriber unmeas_sub_;
        ros::Publisher posSp_pub_;
        ros::Publisher velSp_pub_;

    public:
        HighLevelControl(ros::NodeHandle *nh);
        ~HighLevelControl();

        /**
         * @brief Set the OFFBOARD mode
        */
        void setOffboardCallback(const std_msgs::Bool::ConstPtr &msg);

        /**
         * @brief - Subscribe the current position and orientation of UAV.
         */
        void currentPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);

        /**
         * @brief - Subscribe the estimation results
         */
        void estimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

        /**
         * @brief - Subscribe the estimate of unmeasured data (e.g. velocity of UGV)
        */
        void estimateUnmeasuredCallback(const geometry_msgs::Twist::ConstPtr &msg);

        /**
         * @brief - Calculate the position controller input/output
         */
        void positionController();

        /**
         * @brief - Publish the setpoint
        */
        void setpointPublisher();

        void update();
};


#endif