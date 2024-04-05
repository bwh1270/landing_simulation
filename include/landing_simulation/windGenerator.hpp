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

#ifndef __WIND_GENERATOR__
#define __WIND_GENERATOR__

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8.h"

using namespace std;
using namespace Eigen;

static constexpr int      MSGLength_ = 7;
static constexpr float    LoopRate_  = 0.1f;
static constexpr float    DefaultWindSpeed_        = 0.f;
static constexpr float    DefaultWindSpeedMax_     = 5.f;
static constexpr float    DefaultWindSpeedVar_     = 0.f;
static const     Vector3f DefaultWindDirection_    = Vector3f(0.f,0.f,1.f);  // along with g
static constexpr float    DefaultWindDirectionVar_ = 0.f;

static constexpr uint8_t LANDING_MAGIC_  = 177;
static constexpr uint8_t REVERSER_MAGIC_ = 119; 

enum class PHASE {
    TRACKING,
    LANDING,
    HOLDING
};

enum class FIELD {
    SPEED,
    SPEEDMAX,
    SPEEDVAR,
    DIRECTION_X,
    DIRECTION_Y,
    DIRECTION_Z,
    DIRECTIONVAR
};

class WindGenerator
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber phaseSub_;
        ros::Publisher  windPub_;
        ros::Timer      windLoopTimer_;

        PHASE phase_{PHASE::TRACKING};

        std_msgs::Float32MultiArray windMsg_;
        FIELD field_{FIELD::SPEED};

        // These init. variables can be chanaged
        string   frame_id_;
        float    windSpeed_;
        float    windSpeedMax_;
        float    windSpeedVar_;
        Vector3f windDirection_;
        float    windDirectionVar_;

        int field2int(FIELD field) { return static_cast<int>(field); };

    protected:
    public:
        WindGenerator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        virtual ~WindGenerator();

        void phaseCb(const std_msgs::UInt8::ConstPtr &msg);
        void windLoopCb(const ros::TimerEvent &event);
};

#endif