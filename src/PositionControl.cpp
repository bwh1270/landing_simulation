#include "landing_simulation/PositionControl.hpp"


PositionControl::PositionControl(ros::NodeHandle *nh)
{
    // Input
    UGVStateSub_ = nh->subscribe("/odom", 5, &PositionControl::UGVStateCb, this);
    // UGVStateSub_ = nh->subscribe("/magpie/estimator/state2", 5, &PositionControl::UGVStateCb, this);
    UAVStateSub_ = nh->subscribe("/mavros/global_position/local", 10, &PositionControl::UAVStateCb, this);

    // Output
    velSetpointPub_    = nh->advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 5);
    gimbalSetpointPub_ = nh->advertise<mavros_msgs::MountControl>("/mavros/mount_control/command", 3);

    // State Feedback
    nh->getParam("height_of_ugv", UGVHeight_);
    desiredAlti_[0] = 3.0 + UGVHeight_;
    desiredAlti_[1] = 1.0 + UGVHeight_;

    // Set Control Gains
    nh->getParam("pdff_kp", PDFFGains_.kp);
    nh->getParam("pdff_kd", PDFFGains_.kd);
    nh->getParam("pdff_kff", PDFFGains_.ki);
    nh->getParam("pid_kp", PIDGains_.kp);
    nh->getParam("pid_ki", PIDGains_.ki);
    nh->getParam("pid_kd", PIDGains_.kd);
    bIsNotFirstLoop_ = false;

    // set saturation limit
    nh->getParam("max_horizontal_vel", limVelHorizontal_);
    nh->getParam("max_upward_vel", limVelUp_);
    nh->getParam("max_downward_vel", limVelDown_);
    
    // Initializing
    init();
    setGimbal();

    // Define timer for constant loop rate
    gimbalTimer_ = nh->createTimer(ros::Duration(0.01), &PositionControl::updateGimbal, this);
    
    run();
}

PositionControl::~PositionControl()
{
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

    ROS_INFO("Position Controller is initialized...");
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


// void PositionControl::constrainXY(Vector2f* v0, Vector2f* v1, float* max)
// {
//     if (((*v0) + (*v1)).norm() <= (*max)) {
//         // vector does not exceed maximum magnitude
//         *v0 = (*v0) + (*v1);

//     } else {
//         *v0 = (*v0).normalized() * (*max);
//     }
// }

// void PositionControl::constrainZ(float* vec, float upMax, float downMax)
// {
//     assert(("upMax and downMax is opposite sign", downMax < 0));

//     if (*vec >= upMax) {
//         *vec = upMax;
    
//     } else if (*vec <= downMax) {
//         *vec = downMax;

//     } else {
//         return;
//     }
// }

// Vector3f PositionControl::convertQuaternionToEuler(Vector4f q)
// {
//     Vector3f euler;

//     // roll (x-axis rotation)
//     float sinr_cosp = 2 * (q(3) * q(0) + q(1) * q(2));
//     float cosr_cosp = 1 - 2 * (q(0) * q(0) + q(1) * q(1));
//     euler(0) = std::atan2(sinr_cosp, cosr_cosp);

//     // pitch (y-axis rotation)
//     float sinp = std::sqrt(1 + 2 * (q(3) * q(1) - q(0) * q(2)));
//     float cosp = std::sqrt(1 - 2 * (q(3) * q(1)) - q(0) * q(2));
//     euler(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

//     // yaw (z-axis rotation)
//     float siny_cosp = 2 * (q(3) * q(2) + q(0) * q(1));
//     float cosy_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
//     euler(2) = std::atan2(siny_cosp, cosy_cosp);

//     return euler;
// }

// Vector4f PositionControl::convertEulerToQuaternion(Vector3f euler) // roll (x), pitch (Y), yaw (z)
// {
//     // Abbreviations for the various angular functions
//     float cr = cos(euler(0) * 0.5);
//     float sr = sin(euler(0) * 0.5);
//     float cp = cos(euler(1) * 0.5);
//     float sp = sin(euler(1) * 0.5);
//     float cy = cos(euler(2) * 0.5);
//     float sy = sin(euler(2) * 0.5);

//     Vector4f q;
//     q(3) = cr * cp * cy + sr * sp * sy;
//     q(0) = sr * cp * cy - cr * sp * sy;
//     q(1) = cr * sp * cy + sr * cp * sy;
//     q(2) = cr * cp * sy - sr * sp * cy;

//     return q;
// }


void PositionControl::updatePDFF()
{   
    if (UGVStateVec_.time_stamp.toSec() >= UAVStateVec_.time_stamp.toSec())
    {
        // clock_t cStart = clock();

        // horizontal control - PD
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

// void PositionControl::updatePID()
// {
//     Vector3f posError = UGVStateVec_.pos - UAVStateVec_.pos;
//     Vector3f velSetpoint;

//     if (bIsNotFirstLoop_) {
//         velSetpoint = posError*PIDGains_.kp + (posError - posErrorOld_)*PIDGains_.kd + posErrorSum_*PIDGains_.ki;

//     } else {
//         velSetpoint = posError * PDFFGains_.kp;
// 		bIsNotFirstLoop_ = true;

//     }
//     Vector2f UAVHoriVelSetpoint_;
//     UAVHoriVelSetpoint_(0) = velSetpoint(0);
//     UAVHoriVelSetpoint_(1) = velSetpoint(1);
//     std::cout << UAVHoriVelSetpoint_(0) << ", " << UAVHoriVelSetpoint_(1) << std::endl;
//     Vector2f dump_;
//     dump_(0) = 0.f;
//     dump_(1) = 0.f;
//     constrainXY(&UAVHoriVelSetpoint_, &dump_, &limVelHorizontal_);
//     constrainZ(&velSetpoint(2), limVelUp_, -limVelDown_);
//     std::cout << UAVHoriVelSetpoint_(0) << ", " << UAVHoriVelSetpoint_(1) << std::endl;
//     geometry_msgs::Twist velSetpointMsg;
//     velSetpointMsg.linear.x = UAVHoriVelSetpoint_(0);
//     velSetpointMsg.linear.y = UAVHoriVelSetpoint_(1);
//     velSetpointMsg.linear.z = 0.0f; //velSetpoint(2);
//     velSetpointPub_.publish(velSetpointMsg);

//     posErrorSum_ += posError;
//     posErrorOld_ = posError;
// }

void PositionControl::updateGimbal(const ros::TimerEvent& event)
{
    // Assumption: Experiment only X axis so, UAV is only pitching.
    Vector3d bodyEuler;
    bodyEuler = convertQuaternionToEuler(UAVStateVec_.q);

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
    gimbalSetpointPub_.publish(gimbalMsg);
}


void PositionControl::run()
{
    ros::Rate rate(100.0);

    while (ros::ok())
    {
        updatePDFF();
        // updatePID();

        ros::spinOnce();
        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_node");
    ros::NodeHandle nh("~");

    std::unique_ptr<PositionControl> position_contrl_obj = std::make_unique<PositionControl>(&nh);

    ros::spin();
    return 0;
}