#include "landing_simulation/AprilTagDetection.hpp"
#include "landing_simulation/Mathematics.hpp"


AprilTagDetection::AprilTagDetection(ros::NodeHandle *nh) :
{
    // Get aprilTags ID
    nh->getParam("big_tag_id", tagID_[0]);
    nh->getParam("small_tag_id", tagID_[1]);
    nh->getParam("debug", debug_);
    std::cout << tagID_[0] << std::endl;
    assert( tagID_[0] == 13 );

    // Get name of all frames
    nh->getParam("inertia_frame", inertiaFrame_);
    nh->getParam("body_frame", bodyFrame_);
    nh->getParam("camera_frame", cameraFrame_);
    nh->getParam("big_aprilTag_frame", bigAprilTagFrame_);
    assert( bodyFrame_ == "base_link" );

    // Input & Output
    UAVPoseSub_      = nh->subscribe("/mavros/global_position/local", 10, &AprilTagDetection::poseCb, this);
    detectionRawSub_ = nh->subscribe("/tag_detections", 10, &AprilTagDetection::aprilTagCb, this);
    relativeOdomPub_ = nh->advertise<nav_msgs::Odometry>("/magpie/perception/relative_pose", 5);

    // Initializing
    DCM_ = Matrix3d::Identity(3,3);
    t_ << 0, 0, 0;
    bIsFirstLoop_ = true;

    ROS_INFO("AprilTag Detection Node is activated..");
}

AprilTagDetection::~AprilTagDetection()
{
}

void AprilTagDetection::poseCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    // body w.r.t. inertia broadcaster
    static tf2_ros::TransformBroadcaster br; 
    geometry_msgs::TransformStamped bodyWithRespectToInertiaMsg;

    bodyWithRespectToInertiaMsg.header.frame_id         = inertiaFrame_;
    bodyWithRespectToInertiaMsg.child_frame_id          = bodyFrame_;
    bodyWithRespectToInertiaMsg.header.stamp            = msg->header.stamp;
    bodyWithRespectToInertiaMsg.transform.translation.x = msg->pose.pose.position.x;
	bodyWithRespectToInertiaMsg.transform.translation.y = msg->pose.pose.position.y;
	bodyWithRespectToInertiaMsg.transform.translation.z = msg->pose.pose.position.z;
	bodyWithRespectToInertiaMsg.transform.rotation.x    = msg->pose.pose.orientation.x;
	bodyWithRespectToInertiaMsg.transform.rotation.y    = msg->pose.pose.orientation.y;
	bodyWithRespectToInertiaMsg.transform.rotation.z    = msg->pose.pose.orientation.z;
	bodyWithRespectToInertiaMsg.transform.rotation.w    = msg->pose.pose.orientation.w;
	br.sendTransform(bodyWithRespectToInertiaMsg);

    Vector4d qi_b;
    Vector3d euler;
    qi_b << msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;
    quaternionToEuler(qi_b, euler);

    // camera w.r.t. body broadcaster
    /** @attention When you can measure the gimbal orientation (e.g., through gimbal imu plugin), you should edit the code below: */
    static tf2_ros::StaticTransformBroadcaster staticBr;
    geometry_msgs::TransformStamped camWithRespectToInertiaMsg;
    camWithRespectToInertiaMsg.header.frame_id         = inertiaFrame_;
    camWithRespectToInertiaMsg.child_frame_id          = cameraFrame_;
    camWithRespectToInertiaMsg.header.stamp            = msg->header.stamp;
    camWithRespectToInertiaMsg.transform.translation.x = 0.1 + msg->pose.pose.position.x;
    camWithRespectToInertiaMsg.transform.translation.y = 0.0 + msg->pose.pose.position.y;
    camWithRespectToInertiaMsg.transform.translation.z = -0.1 + msg->pose.pose.position.z;
    tf2::Quaternion q;
    q.setRPY(-3.1415, 0, -1.57+euler(2));
	camWithRespectToInertiaMsg.transform.rotation.x = q.x();
	camWithRespectToInertiaMsg.transform.rotation.y = q.y();
	camWithRespectToInertiaMsg.transform.rotation.z = q.z();
	camWithRespectToInertiaMsg.transform.rotation.w = q.w();
	staticBr.sendTransform(camWithRespectToInertiaMsg);

    Vector4d qi_c;
    qi_c << q.w(), q.x(), q.y(), q.z();
    quaternionToDCM(qi_c, DCM_);
    t_(0) = msg->pose.pose.position.x + 0.1;
    t_(1) = msg->pose.pose.position.y;
    t_(2) = msg->pose.pose.position.z - 0.1;
}

/**
 *  @brief - Transforming the relative pose w.r.t. camera frame to body frame.
 *           So if the gimbal have to be controlled, camera frame - gimbal frame - body frame relation should be published by TF.
 * */
void AprilTagDetection::aprilTagCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{   
    nav_msgs::Odometry relativeOdomMsg;  // output - relative position/velocity of UGV with respect to UAV in {i}
    int detectedPoints = msg->detections.size();

    if (detectedPoints > 0)
    {
        for (int i=0; i<detectedPoints; i++)
        {
            if (msg->detections[i].id[0] == tagID_[0])
            {
                try {
                    //transform the relative position of UGV with respect to UAV in {c} into {i}
                    Vector3d pc_ab, pi_ab;
                    Matrix4d T;
                    pc_ab(0) = msg->detections[i].pose.pose.pose.position.x;
                    pc_ab(1) = msg->detections[i].pose.pose.pose.position.y;
                    pc_ab(2) = msg->detections[i].pose.pose.pose.position.z;
                    setMatrixT(DCM_, t_, T);
                    rotateVector(T, pc_ab, pi_ab);
                    
                    if (debug_) {
                        ROS_INFO("pc_ab: [%.3f, %.3f, %.3f]", pc_ab(0), pc_ab(1), pc_ab(2));
                        std::cout << "DCM: " << std::endl;
                        std::cout << DCM_ << std::endl;
                        std::cout << "T: " << std::endl;
                        std::cout << T << std::endl;
                        ROS_INFO("pi_ab: [%.3f, %.3f, %.3f]", pi_ab(0), pi_ab(1), pi_ab(2));
                    }

                    // for publishing
                    double deltaT = ros::Time::now().toSec()-timeStamp_;
                    if (deltaT == 0.0) {
                            deltaT = deltaT_old_;
                    }

                    if (bIsFirstLoop_) {
                        pi_ab_old_ = pi_ab;
                        bIsFirstLoop_ = false;
                    }

                    relativeOdomMsg.header.frame_id = inertiaFrame_;
                    relativeOdomMsg.header.stamp    = ros::Time::now();
                    relativeOdomMsg.pose.pose.position.x = pi_ab(0);
                    relativeOdomMsg.pose.pose.position.y = pi_ab(1);
                    relativeOdomMsg.pose.pose.position.z = pi_ab(2);
                    relativeOdomMsg.twist.twist.linear.x = (pi_ab(0) - pi_ab_old_(0)) / deltaT;
                    relativeOdomMsg.twist.twist.linear.y = (pi_ab(1) - pi_ab_old_(1)) / deltaT;
                    relativeOdomMsg.twist.twist.linear.z = (pi_ab(2) - pi_ab_old_(2)) / deltaT;
                    relativeOdomPub_.publish(relativeOdomMsg);

                    // for next loop
                    if (deltaT != 0.0) {
                        deltaT_old_ = deltaT;
                    }
                    timeStamp_ = ros::Time::now().toSec();
                    pi_ab_old_ = pi_ab;

                } catch (tf2::TransformException &ex) {
                    ROS_WARN("No valid TF for the required tag %d", tagID_[0]);
                    return;
                }
            }
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_node");
    ros::NodeHandle nh("~");

    std::unique_ptr<AprilTagDetection> apriltag_detection_obj = std::make_unique<AprilTagDetection>(&nh);
    
    ros::spin();
    return 0;
}
