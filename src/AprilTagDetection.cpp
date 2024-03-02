#include "landing_simulation/AprilTagDetection.hpp"
#include "landing_simulation/Mathematics.hpp"


AprilTagDetection::AprilTagDetection(ros::NodeHandle *nh) :
tfListener_(tfBuffer_)
{
    // Get aprilTags ID
    nh->getParam("big_tag_id", tagID_[0]);
    nh->getParam("small_tag_id", tagID_[1]);
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
    relativePosePub_ = nh->advertise<nav_msgs::Odometry>("/magpie/perception/relative_pose", 5);

    q_ << 0, 0, 0, 0;
    bIsFirstLoop_ = true;
    priorPos_ << 0, 0, 0;

    ROS_INFO("AprilTag Detection Node is activated..");
}

AprilTagDetection::~AprilTagDetection()
{
}

void AprilTagDetection::poseCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    q_(0) = msg->pose.pose.orientation.w;
    q_(1) = msg->pose.pose.orientation.x;
    q_(2) = msg->pose.pose.orientation.y;
    q_(3) = msg->pose.pose.orientation.z;

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

    // camera w.r.t. body broadcaster
    static tf2_ros::StaticTransformBroadcaster staticBr;
    geometry_msgs::TransformStamped camWithRespectToBodyMsg;
    camWithRespectToBodyMsg.header.frame_id         = bodyFrame_;
    camWithRespectToBodyMsg.child_frame_id          = cameraFrame_;
    camWithRespectToBodyMsg.header.stamp            = msg->header.stamp;
    camWithRespectToBodyMsg.transform.translation.x = 0.1;
    camWithRespectToBodyMsg.transform.translation.y = 0.0;
    camWithRespectToBodyMsg.transform.translation.z = -0.1;
    tf2::Quaternion q;
    q.setRPY(-3.1415, 0, -1.57);
	camWithRespectToBodyMsg.transform.rotation.x = q.x();
	camWithRespectToBodyMsg.transform.rotation.y = q.y();
	camWithRespectToBodyMsg.transform.rotation.z = q.z();
	camWithRespectToBodyMsg.transform.rotation.w = q.w();
	staticBr.sendTransform(camWithRespectToBodyMsg);
}

/**
 *  @brief - Transforming the relative pose w.r.t. camera frame to body frame.
 *           So if the gimbal have to be controlled, camera frame - gimbal frame - body frame relation should be published by TF.
 * */
void AprilTagDetection::aprilTagCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{   
    Vector4d p, q;
    Matrix4d q_M, pM, qM;
    geometry_msgs::PoseStamped detectedMsg;
    geometry_msgs::PoseStamped bodyPoseMsg;
    nav_msgs::Odometry relativePoseMsg;
    int detectedPoints = msg->detections.size();

    if (detectedPoints > 0)
    {
        for (int i=0; i<detectedPoints; i++)
        {
            if (msg->detections[i].id[0] == tagID_[0])
            {
                try {
                    detectedMsg.header.frame_id = cameraFrame_;
                    detectedMsg.header.stamp = msg->detections[i].pose.header.stamp;
                    detectedMsg.pose.position.x = msg->detections[i].pose.pose.pose.position.x;
                    detectedMsg.pose.position.y = msg->detections[i].pose.pose.pose.position.y;
                    detectedMsg.pose.position.z = msg->detections[i].pose.pose.pose.position.z;
                    detectedMsg.pose.orientation.x = msg->detections[i].pose.pose.pose.orientation.x;
                    detectedMsg.pose.orientation.y = msg->detections[i].pose.pose.pose.orientation.y;
                    detectedMsg.pose.orientation.z = msg->detections[i].pose.pose.pose.orientation.z;
                    detectedMsg.pose.orientation.w = msg->detections[i].pose.pose.pose.orientation.w;

                    bodyPoseMsg = tfBuffer_.transform(detectedMsg, bodyFrame_, ros::Duration(1.0));
                    
                    // convert relative pose w.r.t. from {b} to {i}
                    p(0) = 0.0;
                    p(1) = bodyPoseMsg.pose.position.x;
                    p(2) = bodyPoseMsg.pose.position.y;
                    p(3) = bodyPoseMsg.pose.position.z;
                    q(0) = bodyPoseMsg.pose.orientation.w;
                    q(1) = bodyPoseMsg.pose.orientation.x;
                    q(2) = bodyPoseMsg.pose.orientation.y;
                    q(3) = bodyPoseMsg.pose.orientation.z;

                    q_M = calQuaternionMatrix(q_);
                    pM  = calQuaternionMatrix(p);
                    qM  = calQuaternionMatrix(q);
                    for (int i=0; i<3; i++) {
                        q_(i+1) = -q_(i+1);
                    }
                    p = q_M*pM*q_;
                    q = q_M*qM*q_;
                    // ROS_INFO("[x,y,z]: [%.3f, %.3f, %.3f]", p(1), p(2), p(3));
                    // ROS_INFO("[w,x,y,z]: [%.3f, %.3f, %.3f, %.3f]", q(0), q(1), q(2), q(3));

                    if (!bIsFirstLoop_) {
                        relativePoseMsg.header.frame_id = inertiaFrame_;
                        relativePoseMsg.header.stamp    = bodyPoseMsg.header.stamp;
                        relativePoseMsg.pose.pose.position.x = p(1);
                        relativePoseMsg.pose.pose.position.y = p(2);
                        relativePoseMsg.pose.pose.position.z = p(3);
                        relativePoseMsg.pose.pose.orientation.w = q(0);
                        relativePoseMsg.pose.pose.orientation.x = q(1);
                        relativePoseMsg.pose.pose.orientation.y = q(2);
                        relativePoseMsg.pose.pose.orientation.z = q(3);

                        double deltaT = ros::Time::now().toSec()-timeStamp_;
                        if (deltaT == 0.0) {
                            deltaT = priorDeltaT_;
                        }
                        relativePoseMsg.twist.twist.linear.x = (p(1) - priorPos_(0)) / deltaT;
                        relativePoseMsg.twist.twist.linear.y = (p(2) - priorPos_(1)) / deltaT;
                        relativePoseMsg.twist.twist.linear.z = (p(3) - priorPos_(2)) / deltaT;
                        relativePosePub_.publish(relativePoseMsg);

                        for (int i=0; i<3; i++) {
                            priorPos_(i) = p(i+1);
                            priorDeltaT_ = deltaT;
                            timeStamp_ = ros::Time::now().toSec();
                        }
                    
                    } else {
                        for (int i=0; i<3; i++) {
                            priorPos_(i) = p(i+1);
                            timeStamp_ = ros::Time::now().toSec();
                        }
                        bIsFirstLoop_ = false;
                    }
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
