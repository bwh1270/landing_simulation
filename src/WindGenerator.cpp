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
 * @brief Wind Generator class  
 *
 * @author Woohyun Byun <imbwh@cau.ac.kr>
 */

#include "landing_simulation/windGenerator.hpp"


WindGenerator::WindGenerator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
nh_(nh),
nh_private_(nh_private)
{
    phaseSub_      = nh_.subscribe("/mavros/phase_transition/transition_flag", 1, &WindGenerator::phaseCb, this, ros::TransportHints().tcpNoDelay());
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
    // phase_ = PHASE::LANDING;
}

WindGenerator::~WindGenerator()
{
}

void
WindGenerator::phaseCb(const std_msgs::UInt8::ConstPtr &msg)
{
    auto x = msg->data;

    if (x == LANDING_MAGIC_) {
        cout << "LANDING DETECTED" << endl;
        cout << "wind-On" << endl;
        phase_ = PHASE::LANDING;
    
    } else if (x == REVERSER_MAGIC_) {
        cout << "HOLDING DETECTED" << endl;
        cout << "wind-Off" << endl;
        phase_ = PHASE::HOLDING;
    }
}

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
            windMsg_.data[field2int(FIELD::SPEED)] = 0.f;
            windPub_.publish(windMsg_);
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
