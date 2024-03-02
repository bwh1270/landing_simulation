#include "landing_simulation/test/high_level_control_class.hpp"


FlightMode::FlightMode(ros::NodeHandle *nh) :
freq_(20.0)
{
	current_state_sub_ = nh->subscribe<mavros_msgs::State>("mavros/state", 10, &FlightMode::currentStateCallback, this);
	set_mode_client_   = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arm_client_        = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	takeoff_client_    = nh->serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	land_client_       = nh->serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
}

FlightMode::~FlightMode()
{
}

void FlightMode::currentStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
	state_ = *msg;
}

void FlightMode::setConnection()
{
	ros::Rate rate(freq_);
	ros::Time beginTime = ros::Time::now();
	ros::Duration duration;
	while (ros::ok() && !state_.connected)
	{
		if (duration.toSec() >= 10) {
			ROS_WARN("UAV is not connected for 10 sec");
		}
		else if (duration.toSec() >= 20) {
			ROS_ERROR("UAV is not connected for 20 sec");
			ROS_ERROR("UAV STARTS LANDING");
			// Land function will be here.
		}
		duration = ros::Time::now() - beginTime;
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("UAV is connected");
}

void FlightMode::setArm()
{
	mavros_msgs::CommandBool armSrv;
	armSrv.request.value = true;

	if (arm_client_.call(armSrv) && armSrv.response.success) {
		ROS_INFO("Arming request is sent");
	} 
	else {
		ROS_ERROR("Arming request is NOT sent");
	}

	ros::Rate rate(freq_);
	ros::Time beginTime = ros::Time::now();
	ros::Duration duration;
	while (ros::ok())
	{
		duration = ros::Time::now() - beginTime;
		if (state_.armed && duration.toSec() >= 5.0) {
			ROS_INFO("UAV armed");
			break;
		}
		else if (duration.toSec() > 20.0) {
			ROS_ERROR("Arming failed");
			return;
		}
		ros::spinOnce();
		rate.sleep();
	}
}

void FlightMode::setOffboardMode()
{
	/* This function cannot change mode to OFFBOARD alone.
	 * If you want to change offboard mode then,
	 * publishing to setpoint topics
	 */
	std::string mode = "OFFBOARD";
	ROS_INFO("Setting mode to %s", mode.c_str());

	mavros_msgs::SetMode modeSrv;
	modeSrv.request.custom_mode = mode;

	if (set_mode_client_.call(modeSrv) && modeSrv.response.mode_sent) {
		ROS_INFO("Mode switch request is sent successfully");
	}
	else {
		ROS_WARN("Mode switch request is not sent");
		return;
	}
}

void FlightMode::land()
{
	mavros_msgs::CommandTOL landSrv;
	landSrv.request.altitude = 0.0;
	
	if (land_client_.call(landSrv) && landSrv.response.success) {
		ROS_INFO("UAV starts landing");
	}
	else {
		ROS_ERROR("UAV can not land");
	}
}



HighLevelControl::HighLevelControl(ros::NodeHandle *nh) :
fmode_obj_(nh),
is_offboard_(false),
is_estimated_data_(false),
uav_frame_id_("base_link"),
rate_(20.0),
first_pass_(true),
debug_(false)
{   
    nh->getParam("control_node/P_gain", pidff_gains_(0));
    nh->getParam("control_node/I_gain", pidff_gains_(1));
    nh->getParam("control_node/D_gain", pidff_gains_(2));
    nh->getParam("control_node/FF_gain", pidff_gains_(3));
    nh->getParam("control_node/frequency", rate_);
    nh->getParam("control_node/print_debug", debug_);

    ff_velocity_.vec(2) = 0.0;

    start_sub_        = nh->subscribe("aims/start", 5, &HighLevelControl::setOffboardCallback, this);
    current_pose_sub_ = nh->subscribe("mavros/global_position/local", 10, &HighLevelControl::currentPoseCallback, this);
    estimate_sub_     = nh->subscribe("kf/estimate", 10, &HighLevelControl::estimateCallback, this);
    unmeas_sub_       = nh->subscribe("kf/estimate_unmeas", 5, &HighLevelControl::estimateUnmeasuredCallback, this);
    posSp_pub_        = nh->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    velSp_pub_        = nh->advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // initControl();
}

HighLevelControl::~HighLevelControl()
{
}

void HighLevelControl::setOffboardCallback(const std_msgs::Bool::ConstPtr &msg)
{
    bool flag = msg->data;
    if (flag)
    {
        // Wait for FCU connection
        fmode_obj_.setConnection();

        // Flight ready
    	fmode_obj_.setOffboardMode();

        is_offboard_ = true;
    }
}

void HighLevelControl::currentPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_current_pose_.time_stamp = msg->header.stamp;
    uav_current_pose_.pos(0) = msg->pose.pose.position.x;
    uav_current_pose_.pos(1) = msg->pose.pose.position.y;
    uav_current_pose_.pos(2) = msg->pose.pose.position.z;

    uav_current_pose_.q(0) = msg->pose.pose.orientation.x;
    uav_current_pose_.q(1) = msg->pose.pose.orientation.y;
    uav_current_pose_.q(2) = msg->pose.pose.orientation.z;
    uav_current_pose_.q(3) = msg->pose.pose.orientation.w;
}

void HighLevelControl::estimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // Input with "kf/estimate" - estimation of the tag position w.r.t. UAV frame(base_link)
    // that means the UAV make the setpoint msg to zero.

    // pose setpoint vector = uav vector + estimate vector
    // So, pose_setpoint_ vector is w.r.t. inertia frame.
    is_estimated_data_ = true;
    pos_setpoint_.time_stamp = ros::Time::now();
    pos_setpoint_.vec(0) = uav_current_pose_.pos(0) + msg->pose.pose.position.x;
    pos_setpoint_.vec(1) = uav_current_pose_.pos(1) + msg->pose.pose.position.y;
    pos_setpoint_.vec(2) = uav_current_pose_.pos(2) + msg->pose.pose.position.z;
}

void HighLevelControl::estimateUnmeasuredCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    ff_velocity_.time_stamp = ros::Time::now();
    ff_velocity_.vec(0) = msg->linear.x;
    ff_velocity_.vec(1) = msg->linear.y;
}

void HighLevelControl::positionController()
{
    // PD+FF controller
    Eigen::Vector3d u;

    // only new data can be setpoint
    if ( (pos_setpoint_.time_stamp.toSec() - uav_current_pose_.time_stamp.toSec()) < (1./rate_) ) // @@Q) Is it possible always?
    {
        if (first_pass_)
        {
            e_ = pos_setpoint_.vec - uav_current_pose_.pos; // error of [x,y,z]  w.r.t. {i}
            u = pidff_gains_(0) * e_;
            vel_setpoint_.vec = u + pidff_gains_(3) * ff_velocity_.vec;
            e_old_ = e_;
            first_pass_ = false;
        }
        else {
            e_ = pos_setpoint_.vec - uav_current_pose_.pos; // error of [x,y,z]  w.r.t. {i}
            // ROS_INFO("error : [%.2f, %.2f, %.2f] ", e_(0), e_(1), e_(2));
            u = pidff_gains_(0) * e_ + pidff_gains_(2)*(e_ - e_old_);
            if (debug_) {
                ROS_INFO("control input : [%.2f, %.2f, %.2f] ", u(0), u(1), u(2));
            }
            vel_setpoint_.vec = u + pidff_gains_(3) * ff_velocity_.vec;
            e_old_ = e_;
    }
    }
    else {
        if (debug_) {
            ROS_WARN("pos setpoint time stamp is slower than uav current pose");
        }
        //vel_setpoint_.vec = Eigen::MatrixXd::Zero(3,1);  // I think this line makes the UAV to oscillation.
    }
}

void HighLevelControl::setpointPublisher()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = vel_setpoint_.vec(0);
    vel_msg.linear.y = vel_setpoint_.vec(1);
    vel_msg.linear.z = 0.0;
    // vel_msg.linear.z = vel_setpoint_.vec(2);
    // ROS_INFO("velSp : [%.2f, %.2f, %.2f] ", vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z);
    velSp_pub_.publish(vel_msg);
}

void HighLevelControl::update()
{
    if (is_offboard_)
    {
        positionController();
        setpointPublisher();
    }
    else {
        if (debug_)
            ROS_WARN("Flight mode is not Offboard. Please set the Offboard mode.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    
    HighLevelControl *control_obj = new HighLevelControl(&nh);
    ROS_INFO("High Level Control Node is activated...!");
    
    
    double freq; // Publishing the setpoint will be at less 5 Hz.
    nh.getParam("control_node/frequency", freq);
    
    ros::Rate rate(freq);
    while (ros::ok()) {
        control_obj->update();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}