#ifndef __APRILTAG_DETECTION__
#define __APRILTAG_DETECTION__

#include <iostream>
#include <string>
#include <cassert>
#include <memory>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "apriltag_ros/AprilTagDetectionArray.h"
using namespace std;
using namespace Eigen;


/** @class AprilTag Detection Class
 * @brief - Publish data measured by aprilTag after transforming it from camera frame {c} to body(uav) frame {b}.
 *          And surely, tf b/w camera and body frame should be published.
 */
class AprilTagDetection
{
    private:
        int tagID_[2];
        bool debug_;

        // For rotations
        Vector3d t_;
        Matrix3d DCM_;

        // For calculating the Velocity Measurement
        Vector3d pi_ab_old_;
        double timeStamp_;
        double deltaT_old_; // prevent dt=0 => vel -> infinity
        bool bIsFirstLoop_;

        // Name of all frames
        std::string inertiaFrame_;
        std::string bodyFrame_;
        std::string cameraFrame_;
        std::string bigAprilTagFrame_;
        
        // Input & Output
        ros::Subscriber UAVPoseSub_;      // To convert {c} w.r.t. {i}, {b}-{i} relation is needed.
        ros::Subscriber detectionRawSub_; // input - relative pose b/w aprilTag and camera w.r.t. camera frame
        ros::Publisher relativeOdomPub_; // output - relative pose b/w aprilTag and UAV w.r.t. body frame

    public:
        AprilTagDetection(ros::NodeHandle* nh);
        ~AprilTagDetection();

        void poseCb(const nav_msgs::Odometry::ConstPtr& msg);
        void aprilTagCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

};

#endif