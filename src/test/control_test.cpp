#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/PositionTarget.h"
#include <memory>


class ControlTest
{
    private:
        ros::Subscriber ugv_sub_;
        ros::Publisher setpoint_pub_;
        uint8_t frame_num_;
        double alt_from_ugv_;

    public:
        ControlTest(ros::NodeHandle *nh);
        ~ControlTest();
    
        void UGVPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

ControlTest::ControlTest(ros::NodeHandle *nh)
{
    frame_num_ = 1; // LOCAL NED
    alt_from_ugv_ = 0.2; //-0.2;

    ugv_sub_ = nh->subscribe("odom", 10, &ControlTest::UGVPoseCallback, this);

    setpoint_pub_ = nh->advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 3);

    ROS_INFO("ControlTest class is initialized.");
}

ControlTest::~ControlTest()
{
}


void ControlTest::UGVPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mavros_msgs::PositionTarget setpoint_msg;
    // setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
    //                           mavros_msgs::PositionTarget::FORCE |
    //                           mavros_msgs::PositionTarget::IGNORE_VZ |
    //                           mavros_msgs::PositionTarget::IGNORE_AFX |
    //                           mavros_msgs::PositionTarget::IGNORE_AFY |
    //                           mavros_msgs::PositionTarget::IGNORE_AFZ;
    setpoint_msg.coordinate_frame = frame_num_;
    setpoint_msg.position.x = msg->pose.pose.position.x;
    setpoint_msg.position.y = msg->pose.pose.position.y;
    setpoint_msg.position.z = msg->pose.pose.position.z + alt_from_ugv_;
    setpoint_msg.velocity.x = msg->twist.twist.linear.x;
    setpoint_msg.velocity.y = msg->twist.twist.linear.y;

    setpoint_pub_.publish(setpoint_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_test_ndoe");
    ros::NodeHandle nh;

    ControlTest control_test_obj = ControlTest(&nh);
    // ControlTest *control_test_obj = new ControlTest(&nh);
    // std::unique_ptr<ControlTest> control_test_obj = std::make_unique<ControlTest>(&nh);

    ros::spin();
    return 0;
}