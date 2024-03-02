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
    relativeOdomPub_ = nh->advertise<nav_msgs::Odometry>("/magpie/perception/relative_pose", 5);

    // Initializing
    UAV_q_ << 0, 0, 0, 0;
    priorUGVPos_ << 0, 0, 0;
    bIsFirstLoop_ = true;

    ROS_INFO("AprilTag Detection Node is activated..");
}

AprilTagDetection::~AprilTagDetection()
{
}

void AprilTagDetection::poseCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    UAV_q_(0) = msg->pose.pose.orientation.w;
    UAV_q_(1) = msg->pose.pose.orientation.x;
    UAV_q_(2) = msg->pose.pose.orientation.y;
    UAV_q_(3) = msg->pose.pose.orientation.z;

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
    Vector4d rel_p_b, rel_q_b;
    Matrix4d UAV_q_M, rel_pM, rel_qM;
    geometry_msgs::PoseStamped detectedMsg;
    geometry_msgs::PoseStamped bodyPoseMsg;
    nav_msgs::Odometry relativeOdomMsg;  // output - relative position/velocity of UGV with respect to UAV in {i}
    int detectedPoints = msg->detections.size();

    if (detectedPoints > 0)
    {
        for (int i=0; i<detectedPoints; i++)
        {
            if (msg->detections[i].id[0] == tagID_[0])
            {
                try {
                    // transform the relative position of UGV with respect to UAV in {c} into in {b}
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
                    
                    // transform the relative position of UGV with respect to UAV in {b} into in {i}
                    rel_p_b(0) = 0.0;
                    rel_p_b(1) = bodyPoseMsg.pose.position.x;
                    rel_p_b(2) = bodyPoseMsg.pose.position.y;
                    rel_p_b(3) = bodyPoseMsg.pose.position.z;
                    rel_q_b(0) = bodyPoseMsg.pose.orientation.w;
                    rel_q_b(1) = bodyPoseMsg.pose.orientation.x;
                    rel_q_b(2) = bodyPoseMsg.pose.orientation.y;
                    rel_q_b(3) = bodyPoseMsg.pose.orientation.z;

                    UAV_q_M = calQuaternionMatrix(UAV_q_);
                    rel_pM  = calQuaternionMatrix(rel_p_b);
                    rel_qM  = calQuaternionMatrix(rel_q_b);
                    for (int i=0; i<3; i++) {
                        UAV_q_(i+1) = -UAV_q_(i+1); // conjugate
                    }
                    rel_pM = UAV_q_M*rel_pM*UAV_q_;
                    rel_qM = UAV_q_M*rel_qM*UAV_q_;
                    // ROS_INFO("[x,y,z]: [%.3f, %.3f, %.3f]", p(1), p(2), p(3));
                    // ROS_INFO("[w,x,y,z]: [%.3f, %.3f, %.3f, %.3f]", q(0), q(1), q(2), q(3));

                    if (!bIsFirstLoop_) {
                        relativeOdomMsg.header.frame_id = inertiaFrame_;
                        relativeOdomMsg.header.stamp    = ros::Time::now();
                        relativeOdomMsg.pose.pose.position.x = rel_pM(1);
                        relativeOdomMsg.pose.pose.position.y = rel_pM(2);
                        relativeOdomMsg.pose.pose.position.z = rel_pM(3);
                        relativeOdomMsg.pose.pose.orientation.w = rel_qM(0);
                        relativeOdomMsg.pose.pose.orientation.x = rel_qM(1);
                        relativeOdomMsg.pose.pose.orientation.y = rel_qM(2);
                        relativeOdomMsg.pose.pose.orientation.z = rel_qM(3);

                        double deltaT = ros::Time::now().toSec()-timeStamp_;
                        if (deltaT == 0.0) {
                            deltaT = priorDeltaT_;
                        }
                        relativeOdomMsg.twist.twist.linear.x = (rel_pM(1) - priorUGVPos_(0)) / deltaT;
                        relativeOdomMsg.twist.twist.linear.y = (rel_pM(2) - priorUGVPos_(1)) / deltaT;
                        relativeOdomMsg.twist.twist.linear.z = (rel_pM(3) - priorUGVPos_(2)) / deltaT;
                        relativeOdomPub_.publish(relativeOdomMsg);

                        for (int i=0; i<3; i++) {
                            priorUGVPos_(i) = rel_pM(i+1);
                            priorDeltaT_ = deltaT;
                            timeStamp_ = ros::Time::now().toSec();
                        }
                    
                    } else {
                        for (int i=0; i<3; i++) {
                            priorUGVPos_(i) = rel_pM(i+1);
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
