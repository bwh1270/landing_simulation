#ifndef __WIND_GENERATOR__
#define __WIND_GENERATOR__

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "landing_simulation/wind.h"

using namespace std;
using namespace Eigen;

static constexpr int      MSGLength_ = 7;
static constexpr float    LoopRate_  = 0.1f;
static constexpr float    DefaultWindSpeed_        = 0.f;
static constexpr float    DefaultWindSpeedMax_     = 5.f;
static constexpr float    DefaultWindSpeedVar_     = 0.f;
static const     Vector3f DefaultWindDirection_    = Vector3f(0.f,0.f,1.f);  // along with g
static constexpr float    DefaultWindDirectionVar_ = 0.f;

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
        // ros::Subscriber phaseSub_;
        ros::Publisher  windPub_;
        ros::Timer      windLoopTimer_;

        PHASE phase_{PHASE::TRACKING};

        // landing_simulation::wind windMsg_;
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

        // void phaseCb(const );
        void windLoopCb(const ros::TimerEvent &event);
};

#endif