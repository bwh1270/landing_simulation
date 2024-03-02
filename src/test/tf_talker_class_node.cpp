#include "landing_simulation/test/tf_talker_class.hpp"


TfTalker::TfTalker(ros::NodeHandle *nh)
{
    nh->getParam("uav_frame", uav_frame_);
    nh->getParam("camera_frame", cam_frame_);
    uav_pose_sub_ = nh->subscribe("/mavros/global_position/local", 10, &TfTalker::tfCallback, this);
}
TfTalker::~TfTalker()
{
}

void TfTalker::tfCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf2_ros::TransformBroadcaster map_to_uav_br;
    geometry_msgs::TransformStamped map_to_uav_msg;

    map_to_uav_msg.header.stamp = ros::Time::now();
    map_to_uav_msg.header.frame_id = "map";
    map_to_uav_msg.child_frame_id = uav_frame_;
    map_to_uav_msg.transform.translation.x = msg->pose.pose.position.x;
	map_to_uav_msg.transform.translation.y = msg->pose.pose.position.y;
	map_to_uav_msg.transform.translation.z = msg->pose.pose.position.z;
	map_to_uav_msg.transform.rotation.x = msg->pose.pose.orientation.x;
	map_to_uav_msg.transform.rotation.y = msg->pose.pose.orientation.y;
	map_to_uav_msg.transform.rotation.z = msg->pose.pose.orientation.z;
	map_to_uav_msg.transform.rotation.w = msg->pose.pose.orientation.w;
	map_to_uav_br.sendTransform(map_to_uav_msg);


    static tf2_ros::StaticTransformBroadcaster uav_to_cam_br;
	geometry_msgs::TransformStamped uav_to_cam_msg;

	uav_to_cam_msg.header.stamp = ros::Time::now();//msg->header.stamp; 
	uav_to_cam_msg.header.frame_id = uav_frame_;
	uav_to_cam_msg.child_frame_id  = cam_frame_;
	uav_to_cam_msg.transform.translation.x = 0.1;
	uav_to_cam_msg.transform.translation.y = 0;
	uav_to_cam_msg.transform.translation.z = -0.1;
	tf2::Quaternion camQ;
	camQ.setRPY(-1.570796, 0, -3.1415);
	uav_to_cam_msg.transform.rotation.x = camQ.x();
	uav_to_cam_msg.transform.rotation.y = camQ.y();
	uav_to_cam_msg.transform.rotation.z = camQ.z();
	uav_to_cam_msg.transform.rotation.w = camQ.w();
	uav_to_cam_br.sendTransform(uav_to_cam_msg);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher_node");
    ros::NodeHandle nh;

    TfTalker *tfTalker = new TfTalker(&nh);

    ros::spin();
    return 0;
}
