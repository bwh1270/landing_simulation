#include "landing_simulation/PositionControl.hpp"


PositionControl::PositionControl(ros::NodeHandle *nh)
{
    // Input
    nh->getParam("gt_or_esti", UGVStateTopic_);
    string globalNS = "/";
    assert( UGVStateTopic_[0] == globalNS[0] );
    UGVStateSub_ = nh->subscribe(UGVStateTopic_, 5, &PositionControl::UGVStateCb, this);
    UAVStateSub_ = nh->subscribe("/mavros/global_position/local", 5, &PositionControl::UAVStateCb, this);

    // Output
    velSetpointPub_    = nh->advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 5);
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
    setGimbal();
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
    if (UGVStateVec_.time_stamp.toSec() >= UAVStateVec_.time_stamp.toSec())
    {
        // clock_t cStart = clock();

        // horizontal control - PD+FF
        Vector2d Exy = (UGVStateVec_.pos - UAVStateVec_.pos);
        Vector2d VxySp;

        // vertical control - PID
        double Ez = desiredAlti_[0] - UAVCurrentAlti_;
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

        geometry_msgs::Twist velSetpointMsg;
        velSetpointMsg.linear.x = VxySp(0);
        velSetpointMsg.linear.y = VxySp(1);
        velSetpointMsg.linear.z = VzSp;
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