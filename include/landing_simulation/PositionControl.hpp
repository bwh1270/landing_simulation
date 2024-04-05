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

#ifndef __POSITION_CONTROL_BWH__
#define __POSITION_CONTROL_BWH__

#include <iostream>
#include <cassert>
#include <string>
#include <cmath>
#include <ctime>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/MountControl.h"

#include "landing_simulation/Mathematics.hpp"
using namespace std;
using namespace Eigen;


// Translation has only horizontal components
struct StateVector
{
    ros::Time time_stamp;
    Vector2d pos;
    Vector2d vel;
    Vector4d q;
};

struct ControlGain
{
    float kp;
    float ki;
    float kd;
};

class PositionControl
{
    private:
        // Input
        string UGVStateTopic_;
        ros::Subscriber UGVStateSub_;
        ros::Subscriber UAVStateSub_;

        // Output
        ros::Publisher velSetpointPub_;
        ros::Publisher gimbalSetpointPub_;
        ros::Timer gimbalTimer_;

        // State Feedback
        StateVector UGVStateVec_; // pose w.r.t. {b}} & vel w.r.t. {i}
        StateVector UAVStateVec_; // w.r.t. {i}
        float UGVHeight_;
        float UAVCurrentAlti_;
        float desiredAlti_[2];

        // Control Gains
        ControlGain PDFFGains_;  // for horizontal components
        ControlGain PIDGains_;   // for vertical component
        Vector2d ExyOld_;
        float EzOld_;
        float EzSum_;
        bool bIsNotFirstLoop_;
        bool IsInitialized_;

        // Saturation Limit
        float limVelHorizontal_;
        float limVelUp_;
        float limVelDown_;
     

    public:
        PositionControl(ros::NodeHandle *nh);
        ~PositionControl();

        // Callback Functinos
        void UGVStateCb(const nav_msgs::Odometry::ConstPtr &msg);
        void UAVStateCb(const nav_msgs::Odometry::ConstPtr &msg);
        
        // Initializing
        void setGimbal();
        void init();

        // Controller
        void updatePDFF();
        // void updateGimbal(const ros::TimerEvent& event);
};

#endif