/*********************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Woohyun Byun.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ********************************************************************************/
/**
 * @brief Position Control (derived) class
 *
 * @author Woohyun Byun <imbwh@cau.ac.kr>
 */

#include "landing_simulation/PositionControl.hpp"


PositionControl::PositionControl(ros::NodeHandle *nh)
{
    // Input
    nh->getParam("gt_or_esti", UGVStateTopic_);
    if (UGVStateTopic_ == "gt") {
        UGVStateTopic_ = "/odom";
        
    } else if (UGVStateTopic_ == "esti") {
        UGVStateTopic_ = "/magpie/estimator/state0";
    }
    UGVStateSub_ = nh->subscribe(UGVStateTopic_, 5, &PositionControl::UGVStateCb, this);
    UAVStateSub_ = nh->subscribe("/mavros/global_position/local", 5, &PositionControl::UAVStateCb, this);

    // Output
    velSetpointPub_    = nh->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 5);
    gimbalSetpointPub_ = nh->advertise<mavros_msgs::MountControl>("/mavros/mount_control/command", 3);
    
    // Define timer for constant loop rate
    // gimbalTimer_ = nh->createTimer(ros::Duration(0.01), &PositionControl::updateGimbal, this);

    // State Feedback
    nh->getParam("height_of_ugv", UGVHeight_);
    desiredAlti_[0] = 3.0 + UGVHeight_; // start tracking altitude
    desiredAlti_[1] = 1.0 + UGVHeight_; // end tracking altitude

    // Set Control Gains
    nh->getParam("pdff_kp", PDFFGains_.kp);
    nh->getParam("pdff_kd", PDFFGains_.kd);
    nh->getParam("pdff_kff", PDFFGains_.ki);
    nh->getParam("pid_kp", PIDGains_.kp);
    nh->getParam("pid_ki", PIDGains_.ki);
    nh->getParam("pid_kd", PIDGains_.kd);
    bIsNotFirstLoop_ = false;
    IsInitialized_   = false;

    // set saturation limit
    nh->getParam("max_horizontal_vel", limVelHorizontal_);
    nh->getParam("max_upward_vel", limVelUp_);
    nh->getParam("max_downward_vel", limVelDown_);
    
    // Initializing
    init();
}

PositionControl::~PositionControl()
{
}

void PositionControl::setGimbal()
{
    mavros_msgs::MountControl gimbalMsg;
    gimbalMsg.header.frame_id = "map"; // it means gimbal orientation is w.r.t. {i}
    gimbalMsg.header.stamp = ros::Time::now();
    gimbalMsg.mode = 2;
    gimbalMsg.pitch = -89.9999;
    gimbalMsg.roll = 0.0;
    gimbalMsg.yaw = 0.0;
    gimbalMsg.altitude = 0.0;
    gimbalMsg.latitude = 0.0;
    gimbalMsg.longitude = 0.0;

    ROS_INFO("Camera Down ...");

    ros::Rate rate(10.0);
    for (int i=0; i<100; i++) {
        gimbalSetpointPub_.publish(gimbalMsg);
        ros::spinOnce();
        rate.sleep();
    }
}

void PositionControl::init()
{
    UGVStateVec_.pos = Vector2d::Zero();
    UGVStateVec_.vel = Vector2d::Zero();
    UGVStateVec_.q = Vector4d::Zero();

    UAVStateVec_.pos = Vector2d::Zero();
    UAVStateVec_.vel = Vector2d::Zero();
    UAVStateVec_.q = Vector4d::Zero();
    UAVCurrentAlti_ = 3.0 + UGVHeight_;

    ExyOld_ = Vector2d::Zero();

    setGimbal();
    IsInitialized_ = true;

    ROS_INFO("Position Controller is initialized...");
}



void PositionControl::UGVStateCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // pose w.r.t. {b}} & vel w.r.t. {i}
    UGVStateVec_.time_stamp = msg->header.stamp;
    UGVStateVec_.pos(0) = msg->pose.pose.position.x;
    UGVStateVec_.pos(1) = msg->pose.pose.position.y;
    UGVStateVec_.vel(0)= msg->twist.twist.linear.x;
    UGVStateVec_.vel(1)= msg->twist.twist.linear.y;

    UGVStateVec_.q(0) = msg->pose.pose.orientation.w;
    UGVStateVec_.q(1) = msg->pose.pose.orientation.x;
    UGVStateVec_.q(2) = msg->pose.pose.orientation.y;
    UGVStateVec_.q(3) = msg->pose.pose.orientation.z;

    if (IsInitialized_) {
        updatePDFF();
    }
}

void PositionControl::UAVStateCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // w.r.t. {i}
    UAVStateVec_.time_stamp = msg->header.stamp;
    UAVStateVec_.pos(0) = msg->pose.pose.position.x;
    UAVStateVec_.pos(1) = msg->pose.pose.position.y;
    UAVCurrentAlti_ = msg->pose.pose.position.z;

    UAVStateVec_.q(0) = msg->pose.pose.orientation.w;
    UAVStateVec_.q(1) = msg->pose.pose.orientation.x;
    UAVStateVec_.q(2) = msg->pose.pose.orientation.y;
    UAVStateVec_.q(3) = msg->pose.pose.orientation.z;
}


void PositionControl::updatePDFF()
{   
    // i think UGVStateVec_ and UAVStateVec_ needs the buffer to find min time difference.
    // And i think not same value of control gains is set for x and y
    // auto dt = UGVStateVec_.time_stamp.toSec() - UAVStateVec_.time_stamp.toSec();
    // cout << dt << endl;
    if (UGVStateVec_.time_stamp.toSec() >= UAVStateVec_.time_stamp.toSec())
    {
        // clock_t cStart = clock();

        // horizontal control - PD+FF
        Vector2d Exy = (UGVStateVec_.pos - UAVStateVec_.pos);
        Vector2d VxySp;

        // vertical control - PID
        double Ez = desiredAlti_[1] - UAVCurrentAlti_;
        double VzSp;

        // ROS_WARN("Error of XY: [%.3f, %.3f]", Exy(0), Exy(1));
        if (bIsNotFirstLoop_) {
            VxySp = Exy * PDFFGains_.kp + (Exy - ExyOld_) * PDFFGains_.kd;
            VzSp  = Ez * PIDGains_.kp + (Ez-EzOld_) * PIDGains_.kd + EzSum_*PIDGains_.ki;

        } else {
            VxySp = Exy * PDFFGains_.kp;
            VzSp  = Ez  * PIDGains_.kp;
            bIsNotFirstLoop_ = true;
        }

        UGVStateVec_.vel *= PDFFGains_.ki;  // in here, ki := kff
        constrainXY(&VxySp, &UGVStateVec_.vel, &limVelHorizontal_);
        constrainZ(&VzSp, limVelUp_, -limVelDown_);

        if ((VzSp == limVelUp_) || (VzSp == -limVelDown_)) {
            EzSum_ = 0; // Re-initializing
        }

        mavros_msgs::PositionTarget velSetpointMsg;
        velSetpointMsg.coordinate_frame = 1;           // FRAME_LOCAL_NED
        velSetpointMsg.type_mask = 0b0000011111000111; // vx, vy, vz, vyaw
        velSetpointMsg.velocity.x = VxySp(0);
        velSetpointMsg.velocity.y = VxySp(1);
        velSetpointMsg.velocity.z = VzSp;
        velSetpointMsg.yaw_rate = 0.;
        velSetpointPub_.publish(velSetpointMsg);

        ExyOld_ = Exy;
        EzOld_  = Ez;
        EzSum_ += Ez;

        // clock_t cEnd = clock();
        // double deltaTimeMs = 1000.0*(cEnd-cStart) / CLOCKS_PER_SEC;
        // ROS_INFO("Execution Time during 'SEARCH' used: [%.3f] [ms]", deltaTimeMs);
    }
}

/** @brief  When you should control the gimbal, complete the below function */ 
// void PositionControl::updateGimbal(const ros::TimerEvent& event)
// {
//     // Assumption: Experiment only X axis so, UAV is only pitching.
//     Vector3d bodyEuler;
//     bodyEuler = convertQuaternionToEuler(UAVStateVec_.q);

//     mavros_msgs::MountControl gimbalMsg;
//     gimbalMsg.header.frame_id = "map"; // it means gimbal orientation is w.r.t. {i}
//     gimbalMsg.header.stamp = ros::Time::now();
//     gimbalMsg.mode = 2;
//     gimbalMsg.pitch = -89.9999;
//     gimbalMsg.roll = 0.0;
//     gimbalMsg.yaw = 0.0;
//     gimbalMsg.altitude = 0.0;
//     gimbalMsg.latitude = 0.0;
//     gimbalMsg.longitude = 0.0;
//     gimbalSetpointPub_.publish(gimbalMsg);
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_node");
    ros::NodeHandle nh("~");

    std::unique_ptr<PositionControl> position_contrl_obj = std::make_unique<PositionControl>(&nh);

    ros::spin();
    return 0;
}