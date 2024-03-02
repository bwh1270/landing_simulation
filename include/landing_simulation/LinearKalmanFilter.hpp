#ifndef __LINEAR_KALMAN_FILTER__
#define __LINEAR_KALMAN_FILTER__

#include <iostream>
#include <cassert>
#include <memory>
#include <cstdlib>
#include <string>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <functional>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "landing_simulation/Mathematics.hpp"
using namespace std;
using namespace Eigen;


/** 
 * @brief - To store the current step KF prediction 
 */
struct KFState
{
    ros::Time time_stamp;
    VectorXd x;  // State estimate (from KF prediction step)
    MatrixXd P;  // State estimate covariance (from KF prediction step)
};

/**
 * @brief - To store current step sensor measurement
 */
struct SensorMeasurement
{
    ros::Time time_stamp;
    VectorXd z;
};


/** @class - Linear Kalman Filter Class
 * @brief -  Based on constant acceleration model and continuous noise model
 */
class LinearKF
{
    private:
        std::string bodyFrame_; // coordinate frame at which tracking is performed
        std::string bigAprilTagFrame_;
        std::string smallAprilTagFrame_;
        float UGVHeight_;

        KFState statePred_; // State and Covariance Prediction
        KFState statePred2_; 
        KFState statePred3_;
        KFState stateForControl_;  // only for prediction
        std::vector<KFState> statePredBuffer_; // buffer to store last predicted x and P for finding closest prediction time
        std::vector<KFState> statePredBuffer2_;
        std::vector<KFState> statePredBuffer3_;
        unsigned int statePredBufferSize_; // length of state buffer

        SensorMeasurement ZMeas_;
        SensorMeasurement ZLastMeas_;
        SensorMeasurement ZMeas2_;
        SensorMeasurement ZLastMeas2_;

        MatrixXd F_;
        MatrixXd F2_;
        MatrixXd F3_;
        MatrixXd Q_;
        MatrixXd Q2_;
        MatrixXd Q3_;
        MatrixXd Qc_;
        MatrixXd H_;
        MatrixXd R_;
        float qu_;            // variance of process noise about UAV(Body)
        float qa_;            // variance of process noise about GV(AprilTag)
        float ru_pxy_;
        float ru_pz_;
        float ru_vxy_;
        float ru_vz_;
        float ru_a_;
        float rau_pxy_;
        float rau_pz_;
        float rau_vxy_;
        float rau_vz_;
        float dt_;            // KF prediction sampling time in seconds
        
        bool bAprilTagIsBig_;      // flag to check whether tag is big or not
        bool bKFIsInitialized_; // flag to start state prediction. 1st measurement
        bool bAprilTagIsMeasured_;
        
        double maxMeasurementOffTime_; // maximum time (in secs) with no measurement before filter is stopped        
        double tagTimeStamp_;

        // Set Matrix
        void setF(MatrixXd* F, double dt);
        void setQ(MatrixXd* Q, double dt);
        void setQc(MatrixXd* Qc, double dt);

        void setH();
        void setR();


        // Kalman Filter Initialization
        void initKF(); // Initialize F_,Q_,H_,R_, initial state and covraiance estimates


        // Add the current x_pred_ and P_red_ state to the end of buffer state_buffer_.
        // It erases the first element in the state_buffer_ vector if its size exceeds state_buffer_size_.
        void updateStatePredBuffer();
        void updateStatePredBuffer2();
        void updateStatePredBuffer3();


        // Perform Kalman Filter prediction step. Result is stored in statePred_.
        /** @return - true if prediction is successful (e.g. state does not explode) */
        bool predict();
        bool predict2();
        bool predict3();
        bool predictAsynchornously(); // only for predict

        // Perform Kalman Filter update step.
        // void update();

        // Publish the Relative Position w.r.t. Inertia and Absolute Velocity of AprilTag w.r.t. Inertia
        // This data will be used into Control UAV and Gimbal.
        void publishState();
        void publishState2();
        void publishState3();
    
        // Input & Output
        ros::Subscriber aprilTagSub_; // to get measurement data from aprilTag
        ros::Publisher statePub_;
        ros::Publisher statePub2_;
        ros::Publisher statePub3_;
        message_filters::Subscriber<nav_msgs::Odometry> GPSsub_;
        message_filters::Subscriber<sensor_msgs::Imu> IMUsub_;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> mySyncPolicy_;
        boost::shared_ptr<message_filters::Synchronizer<mySyncPolicy_>> sync_;

        ros::Timer predictTimer_;
        void predictLoop(const ros::TimerEvent& event);

        // Update measurements
        void aprilTagCb(const nav_msgs::Odometry::ConstPtr& msg);
        void GPSIMUCb(const nav_msgs::Odometry::ConstPtr& gps, const sensor_msgs::Imu::ConstPtr& imu);

    public:
        LinearKF(ros::NodeHandle *nh);
        ~LinearKF();

        
};

#endif