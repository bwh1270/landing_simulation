#include "landing_simulation/windGenerator.hpp"


WindGenerator::WindGenerator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
nh_(nh),
nh_private_(nh_private)
{
    // phaseSub_      = nh_.subscribe("/not_yet", 1, &WindGenerator::phaseCallback, this, ros::TransprotHints().tcpNoDelay());
    windPub_       = nh_.advertise<std_msgs::Float32MultiArray>("/wind", 10);
    windLoopTimer_ = nh_.createTimer(ros::Duration(LoopRate_), &WindGenerator::windLoopCb, this);

    nh_private_.param<string>("frame_id",           frame_id_,        "world");
    nh_private_.param<float>("init_wind_speed",     windSpeed_,        DefaultWindSpeed_);
    nh_private_.param<float>("init_wind_speed_max", windSpeedMax_,     DefaultWindSpeedMax_);
    nh_private_.param<float>("init_wind_speed_var", windSpeedVar_,     DefaultWindSpeedVar_);
    windDirection_ = DefaultWindDirection_;
    nh_private_.param<float>("init_wind_dir_var",   windDirectionVar_, DefaultWindDirectionVar_);
    
    // Init. Wind Message 
    windMsg_.data.clear();
    windMsg_.data.resize(MSGLength_);
    windMsg_.data[field2int(FIELD::SPEED)]        = windSpeed_;
    windMsg_.data[field2int(FIELD::SPEEDMAX)]     = windSpeedMax_;
    windMsg_.data[field2int(FIELD::SPEEDVAR)]     = windSpeedVar_;
    windMsg_.data[field2int(FIELD::DIRECTION_X)]  = windDirection_(0);
    windMsg_.data[field2int(FIELD::DIRECTION_Y)]  = windDirection_(1);
    windMsg_.data[field2int(FIELD::DIRECTION_Z)]  = windDirection_(2);
    windMsg_.data[field2int(FIELD::DIRECTIONVAR)] = windDirectionVar_;

    // For test
    phase_ = PHASE::LANDING;
}

WindGenerator::~WindGenerator()
{
}

// void
// WindGenerator::phaseCb(const )
// {
        // WHEN LANDING
        // phase_ = static_cast<PHASE>(2);
// }

void
WindGenerator::windLoopCb(const ros::TimerEvent &event)
{
    switch (phase_)
    {
        case PHASE::LANDING:
            windMsg_.data[field2int(FIELD::SPEED)] = 5.f;
            windPub_.publish(windMsg_);
            break;
        
        case PHASE::HOLDING:
            break;
        
        default:
            break;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wind_generator_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    WindGenerator* windy = new WindGenerator(nh, nh_private);

    ros::spin();
    return 0;
}
