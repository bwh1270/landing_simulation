#include "landing_simulation/LinearKalmanFilter.hpp"


LinearKF::LinearKF(ros::NodeHandle *nh) :
statePredBufferSize_(40),
bAprilTagIsBig_(true)
{
   // Get Parameters
   nh->getParam("body_frame", bodyFrame_);
   nh->getParam("big_aprilTag_frame", bigAprilTagFrame_);
   nh->getParam("small_aprilTag_frame", smallAprilTagFrame_);
   assert( bodyFrame_ == "base_link" );

   nh->getParam("dt", dt_);
   nh->getParam("qb_var", qb_);
   nh->getParam("qa_var", qa_);
   nh->getParam("rm_from_GPS", rfromGPS_);
   nh->getParam("rm_from_IMU", rfromIMU_);
   nh->getParam("rm_from_aprilTag", rfromAprilTag_);

   nh->getParam("max_measurement_off_time", maxMeasurementOffTime_);


   // Input & Output
   aprilTagSub_ = nh->subscribe("/magpie/perception/relative_pose", 10, &LinearKF::aprilTagCallback, this);
   UAVGPSSub_   = nh->subscribe("/mavros/global_position/local", 10, &LinearKF::GPSCallback, this);
   UAVIMUSub_   = nh->subscribe("/mavros/imu/data", 10, &LinearKF::IMUCallback, this);
   statePub_    = nh->advertise<nav_msgs::Odometry>("/magpie/estimator/state", 2);

   ROS_INFO("Linear Kalman Filter Node is activated...");
   ROS_INFO("State buffer length corresponds to %.3f seconds", dt_*(double)statePredBufferSize_);
   
   initKF();

   // Define timer for constant loop rate
   // maxAprilTagNotDetectedTime_ = nh->createTimer(ros::Duration(dt_), &LinearKF::compareCurrentAndOldTagTime, this);
}

LinearKF::~LinearKF()
{
}

/** @brief - This model is for constant acceleration */
void LinearKF::setF()
{
   F_.resize(18,18);
   F_ = MatrixXd::Identity(18,18);
   
   F_(0,3) = dt_;
   F_(0,6) = (dt_*dt_)/2.;
   F_(1,4) = dt_;
   F_(1,7) = (dt_*dt_)/2.;
   F_(2,5) = dt_;
   F_(2,8) = (dt_*dt_)/2.;
   F_(3,6) = dt_;
   F_(4,7) = dt_;
   F_(5,8) = dt_;

   F_(9.12) = dt_;
   F_(9,15) = (dt_*dt_)/2.;
   F_(10,13) = dt_;
   F_(10,16) = (dt_*dt_)/2.;
   F_(11,14) = dt_;
   F_(11,17) = (dt_*dt_)/2.;
   F_(12,15) = dt_;
   F_(13,16) = dt_;
   F_(14,17) = dt_;
}

void LinearKF::setQ()
{
   Q_.resize(18,18);
   Q_ = MatrixXd::Zero(18,18);

   Q_(0,0) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // x with x (UAV or Body)
   Q_(0,3) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // x with vx
   Q_(0,6) = (qb_)*(dt_*dt_*dt_)/6.;          // x with ax
   Q_(1,1) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // y with y
   Q_(1,4) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // y with vy
   Q_(1,7) = (qb_)*(dt_*dt_*dt_)/6.;          // y with ay
   Q_(2,2) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // z with z
   Q_(2,5) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // z with vz
   Q_(2,8) = (qb_)*(dt_*dt_*dt_)/6.;          // z with az
   Q_(3,0) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // vx with x
   Q_(3,3) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // vx with vx
   Q_(3,6) = (qb_)*(dt_*dt_*dt_)/6.;          // vx with ax
   Q_(4,1) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // vy with y
   Q_(4,4) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // vy with vy
   Q_(4,7) = (qb_)*(dt_*dt_*dt_)/6.;          // vy with ay
   Q_(5,2) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // vz with z
   Q_(5,5) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // vz with vz
   Q_(5,8) = (qb_)*(dt_*dt_*dt_)/6.;          // vz with az
   Q_(6,0) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // ax with x
   Q_(6,3) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // ax with vx
   Q_(6,6) = (qb_)*(dt_*dt_*dt_)/6.;          // ax with ax
   Q_(7,1) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // ay with y
   Q_(7,4) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // ay with vy
   Q_(7,7) = (qb_)*(dt_*dt_*dt_)/6.;          // ay with ay
   Q_(8,2) = (qb_)*(dt_*dt_*dt_*dt_*dt_)/20.; // az with z
   Q_(8,5) = (qb_)*(dt_*dt_*dt_*dt_)/8.;      // az with vz
   Q_(8,8) = (qb_)*(dt_*dt_*dt_)/6.;          // az with az
   
   Q_(9,9) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // x with x (GV or AprilTag)
   Q_(9,12) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // x with vx
   Q_(9,15) = (qa_)*(dt_*dt_*dt_)/6.;          // x with ax
   Q_(10,10) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // y with y
   Q_(10,13) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // y with vy
   Q_(10,16) = (qa_)*(dt_*dt_*dt_)/6.;          // y with ay
   Q_(11,11) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // z with z
   Q_(11,14) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // z with vz
   Q_(11,17) = (qa_)*(dt_*dt_*dt_)/6.;          // z with az
   Q_(12,9) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // vx with x
   Q_(12,12) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // vx with vx
   Q_(12,15) = (qa_)*(dt_*dt_*dt_)/6.;          // vx with ax
   Q_(13,10) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // vy with y
   Q_(13,13) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // vy with vy
   Q_(13,16) = (qa_)*(dt_*dt_*dt_)/6.;          // vy with ay
   Q_(14,11) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // vz with z
   Q_(14,14) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // vz with vz
   Q_(14,17) = (qa_)*(dt_*dt_*dt_)/6.;          // vz with az
   Q_(15,9) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // ax with x
   Q_(15,12) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // ax with vx
   Q_(15,15) = (qa_)*(dt_*dt_*dt_)/6.;          // ax with ax
   Q_(16,10) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // ay with y
   Q_(16,13) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // ay with vy
   Q_(16,16) = (qa_)*(dt_*dt_*dt_)/6.;          // ay with ay
   Q_(17,11) = (qa_)*(dt_*dt_*dt_*dt_*dt_)/20.; // az with z
   Q_(17,14) = (qa_)*(dt_*dt_*dt_*dt_)/8.;      // az with vz
   Q_(17,17) = (qa_)*(dt_*dt_*dt_)/6.;          // az with az
}

void LinearKF::setHfromGPS()
{
   HfromGPS_.resize(6,18);
   HfromGPS_ = MatrixXd::Zero(6,18);
   HfromGPS_(0,0) = 1.0; // observing x of uav
   HfromGPS_(1,1) = 1.0; // observing y 
   HfromGPS_(2,2) = 1.0; // observing z 
   HfromGPS_(3,3) = 1.0; // observing vx
   HfromGPS_(4,4) = 1.0; // observing vy
   HfromGPS_(5,5) = 1.0; // observing vz
}

void LinearKF::setHfromIMU()
{
   HfromIMU_.resize(3,18);
   HfromIMU_ = MatrixXd::Zero(3,18);
   HfromIMU_(0,6) = 1.0; // observing ax
   HfromIMU_(1,7) = 1.0; // observing ay
   HfromIMU_(2,8) = 1.0; // observing az
}

void LinearKF::setHfromGPSIMU()
{
   HfromGPSIMU_.resize(9,18);
   HfromGPSIMU_ = MatrixXd::Zero(9,18);
   HfromGPSIMU_(0,0) = 1.0;
   HfromGPSIMU_(1,1) = 1.0;
   HfromGPSIMU_(2,2) = 1.0;
   HfromGPSIMU_(3,3) = 1.0;
   HfromGPSIMU_(4,4) = 1.0;
   HfromGPSIMU_(5,5) = 1.0;
   HfromGPSIMU_(6,6) = 1.0;
   HfromGPSIMU_(7,7) = 1.0;
   HfromGPSIMU_(8,8) = 1.0;
}


void LinearKF::setHfromAprilTag()
{
   HfromAprilTag_.resize(3, 18);
   HfromAprilTag_ = MatrixXd::Zero(3,18);
   HfromAprilTag_(0,0) = 1.0;  // observing relative x
   HfromAprilTag_(0,6) = -1.0; 
   HfromAprilTag_(1,1) = 1.0;  // observing relative y
   HfromAprilTag_(1,7) = -1.0; 
   HfromAprilTag_(2,2) = 1.0;  // observing relative z
   HfromAprilTag_(2,8) = -1.0; 
}

void LinearKF::setRfromGPS()
{
   RfromGPS_.resize(6,6);
   RfromGPS_ = MatrixXd::Identity(6,6);
   RfromGPS_ = rfromGPS_*rfromGPS_*RfromGPS_;  // multiply by observation noise variance
}

void LinearKF::setRfromIMU()
{
   RfromIMU_.resize(3,3);
   RfromIMU_ = MatrixXd::Identity(3,3);
   RfromIMU_ = rfromIMU_*rfromIMU_*RfromIMU_;  // multiply by observation noise variance
}

void LinearKF::setRfromAprilTag()
{
   RfromAprilTag_.resize(3,3);
   RfromAprilTag_ = MatrixXd::Identity(3,3);
   RfromAprilTag_ = rfromAprilTag_*RfromAprilTag_; // multiply by observation noise variance
}

void LinearKF::setRfromGPSIMU()
{
   RfromGPSIMU_.resize(9,9);
   RfromGPSIMU_ = MatrixXd::Identity(9,9);
   for (int i=0; i<6; i++) {
      RfromGPSIMU_(i,i) = rfromGPS_*RfromGPSIMU_(i,i);
   }
   for (int i=6; i<9; i++) {
      RfromGPSIMU_(i,i) = rfromIMU_*RfromGPSIMU_(i,i);
   }
}

void LinearKF::initKF()
{
   setF();              // Initialize state transition matrix
   setQ();              // Initialize process noise covariance
   setHfromGPS();       // Initialize observation matrix of GPS
   setHfromIMU();       // Initialize observation matrix of IMU
   setHfromAprilTag();  // Initialize observation matrix of AprilTag
   setRfromGPS();       // Initialize observation noise covariance of GPS
   setRfromIMU();       // Initialize observation noise covariance of IMU
   setRfromAprilTag();  // Initialize observation noise covariance of AprilTag
   setHfromGPSIMU();
   setRfromGPSIMU();

   // initial state estimate (0|0 step)
   statePred_.time_stamp = ros::Time::now();
   statePred_.x = MatrixXd::Zero(18,1);
   statePred_.P = Q_;

   for (int i=3; i<9; i++) {
      statePred_.x(i,0) = 0.0000001;   // vel and acc of UAV
      statePred_.x(i+9,0) = 0.000001;  // vel and acc of GV
   }

   statePredBuffer_.clear();  // Clear state buffer

   // initial three measurement vectors (0|0 step)
   // ZMeasFromGPS_.time_stamp = 0.0; // Due to this lines, assertion error occurs. (kf node data is made before now)
   // ZMeasFromIMU_.time_stamp = 0.0;
   // ZMeasFromAprilTag_.time_stamp = 0.0;
   ZMeasFromGPS_.z.resize(6,1);
   ZMeasFromIMU_.z.resize(3,1);
   ZMeasFromAprilTag_.z.resize(3,1);
   ZMeasFromGPSIMU_.z.resize(9,1);
   ZMeasFromGPS_.z = MatrixXd::Zero(6,1);       // measured from GPS
   ZMeasFromIMU_.z = MatrixXd::Zero(3,1);       // measured from IMU
   ZMeasFromAprilTag_.z = MatrixXd::Zero(3,1);  // maasured from AprilTag
   ZMeasFromGPSIMU_.z = MatrixXd::Zero(9,1);
   ZLastMeasFromGPS_ = ZMeasFromGPS_;
   ZLastMeasFromIMU_ = ZMeasFromIMU_;
   ZLastMeasFromAprilTag_ = ZMeasFromAprilTag_;
   ZLastMeasFromGPSIMU_ =  ZMeasFromGPSIMU_;

   // First Prediction for X_{1|0} with X_{0|0}
   predict();
   bKFIsInitialized_ = true;

   ROS_INFO("Linear Kalman Filter is initialized.");
}


void LinearKF::aprilTagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   Vector3d pos;
   pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
   if (pos.norm() > 10000.0){
      ROS_ERROR("[KF aprilTagCallback] Infinite measurement value. Ignoring measurement.");
      return;
   }
   
   // if (bKFIsInitialized_)
   // {  
   //    // step 1: measure
   //    ZMeasFromAprilTag_.time_stamp = msg->header.stamp;
   //    ZMeasFromAprilTag_.z(0,0) = msg->pose.position.x;
   //    ZMeasFromAprilTag_.z(1,0) = msg->pose.position.y;
   //    ZMeasFromAprilTag_.z(2,0) = msg->pose.position.z;
   //    assert( ZMeasFromAprilTag_.time_stamp.toSec() >= ZLastMeasFromAprilTag_.time_stamp.toSec() );

   //    // prevent the time confusing due to calculation delay
   //    for (int i=0; i<statePredBuffer_.size(); i++)
   //    {
   //       auto deltaT = ZMeasFromAprilTag_.time_stamp.toSec() - statePredBuffer_[i].time_stamp.toSec();
   //       if ((deltaT <= dt_) && (deltaT >= 0))
   //       {
   //          statePred_.time_stamp = statePredBuffer_[i].time_stamp;
   //          statePred_.x = statePredBuffer_[i].x;
   //          statePred_.P = statePredBuffer_[i].P;
   //          statePredBuffer_.erase(statePredBuffer_.begin(),statePredBuffer_.begin()+i+1); // remove old states
   //          break;
   //       }
   //    }

   //    // step 2: update
   //    // compute measurement residual (innovation)
   //    auto y = ZMeasFromAprilTag_.z - (HfromAprilTag_ * statePred_.x);

   //    // Innovation covariance
   //    auto S = (HfromAprilTag_ * statePred_.P * HfromAprilTag_.transpose()) + RfromAprilTag_;

   //    // optimal Kalman gain
   //    auto K = statePred_.P * HfromAprilTag_.transpose() * S.inverse();

   //    // Updated state estimate and it's covariacne
   //    statePred_.x = statePred_.x + (K*y);
   //    statePred_.P = statePred_.P - (K*HfromAprilTag_*statePred_.P);

   //    ZLastMeasFromAprilTag_.time_stamp = ZMeasFromAprilTag_.time_stamp;
   //    ZLastMeasFromAprilTag_.z = ZMeasFromAprilTag_.z;

   //    // step 3: predict
   //    if (!predict()) {
   //       ROS_WARN("AprilTag Precition");
   //       return;
   //    }

   //    // ----------- when AprilTag is not detected, then ... what code is needed?
   //    //if ((ros::Time::now().toSec() - ZMeasFromAprilTag_.time_stamp.toSec()) > maxMeasurementOffTime_)
   // }
}

void LinearKF::GPSCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   Vector3d pos;
   Vector3d vel;
   pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
   vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
   if ((pos.norm() > 10000.0) || (vel.norm() > 10000.0)) {
      ROS_ERROR("[KF GPSCallback] Infinite measurement value. Ignoring measurement.");
      return;
   }

   if (bKFIsInitialized_)
   {  
      // step 1: measure
      ZMeasFromGPSIMU_.time_stamp = msg->header.stamp;
      ZMeasFromGPSIMU_.z(0) = msg->pose.pose.position.x;
      ZMeasFromGPSIMU_.z(1) = msg->pose.pose.position.y;
      ZMeasFromGPSIMU_.z(2) = msg->pose.pose.position.z;
      ZMeasFromGPSIMU_.z(3) = msg->twist.twist.linear.x;
      ZMeasFromGPSIMU_.z(4) = msg->twist.twist.linear.y;
      ZMeasFromGPSIMU_.z(5) = msg->twist.twist.linear.z;
      ZMeasFromGPSIMU_.z(6) = ZMeasFromIMU_.z(0);
      ZMeasFromGPSIMU_.z(7) = ZMeasFromIMU_.z(1);
      ZMeasFromGPSIMU_.z(8) = ZMeasFromIMU_.z(2);
      assert( ZMeasFromGPSIMU_.time_stamp.toSec() >= ZLastMeasFromGPSIMU_.time_stamp.toSec() );

      // prevent the time confusing due to calculation delay
      for (int i=0; i<statePredBuffer_.size(); i++)
      {
         auto deltaT = ZMeasFromGPSIMU_.time_stamp.toSec() - statePredBuffer_[i].time_stamp.toSec();
         if ((deltaT <= dt_) && (deltaT >= 0))
         {
            statePred_.time_stamp = statePredBuffer_[i].time_stamp;
            statePred_.x = statePredBuffer_[i].x;
            statePred_.P = statePredBuffer_[i].P;
            statePredBuffer_.erase(statePredBuffer_.begin(),statePredBuffer_.begin()+i+1); // remove old states
            break;
         }
      }

      // step 2: update
      // compute measurement residual (innovation)
      auto y = ZMeasFromGPSIMU_.z - (HfromGPSIMU_ * statePred_.x);

      // Innovation covariance
      auto S = (HfromGPSIMU_ * statePred_.P * HfromGPSIMU_.transpose()) + RfromGPSIMU_;

      // optimal Kalman gain
      auto K = statePred_.P * HfromGPSIMU_.transpose() * S.inverse();

      // Updated state estimate and it's covariacne
      statePred_.x = statePred_.x + (K*y);
      statePred_.P = statePred_.P - (K*HfromGPSIMU_*statePred_.P);

      ZLastMeasFromGPSIMU_.time_stamp = ZMeasFromGPSIMU_.time_stamp;
      ZLastMeasFromGPSIMU_.z = ZMeasFromGPSIMU_.z;
      ZLastMeasFromGPS_.time_stamp = ZMeasFromGPS_.time_stamp;
      ZLastMeasFromGPS_.z = ZMeasFromGPS_.z;

      // step 3: predict
      if (!predict()) {
         ROS_WARN("GPS Precition");
         return;
      }
   }
}

// void LinearKF::GPSCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//    Vector3d pos;
//    Vector3d vel;
//    pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
//    vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
//    if ((pos.norm() > 10000.0) || (vel.norm() > 10000.0)) {
//       ROS_ERROR("[KF GPSCallback] Infinite measurement value. Ignoring measurement.");
//       return;
//    }

//    if (bKFIsInitialized_)
//    {  
//       // step 1: measure
//       ZMeasFromGPS_.time_stamp = msg->header.stamp;
//       ZMeasFromGPS_.z(0) = msg->pose.pose.position.x;
//       ZMeasFromGPS_.z(1) = msg->pose.pose.position.y;
//       ZMeasFromGPS_.z(2) = msg->pose.pose.position.z;
//       ZMeasFromGPS_.z(3) = msg->twist.twist.linear.x;
//       ZMeasFromGPS_.z(4) = msg->twist.twist.linear.y;
//       ZMeasFromGPS_.z(5) = msg->twist.twist.linear.z;
//       assert( ZMeasFromGPS_.time_stamp.toSec() >= ZLastMeasFromGPS_.time_stamp.toSec() );
   
//       // prevent the time confusing due to calculation delay
//       for (int i=0; i<statePredBuffer_.size(); i++)
//       {
//          auto deltaT = ZMeasFromGPS_.time_stamp.toSec() - statePredBuffer_[i].time_stamp.toSec();
//          if ((deltaT <= dt_) && (deltaT >= 0))
//          {
//             statePred_.time_stamp = statePredBuffer_[i].time_stamp;
//             statePred_.x = statePredBuffer_[i].x;
//             statePred_.P = statePredBuffer_[i].P;
//             statePredBuffer_.erase(statePredBuffer_.begin(),statePredBuffer_.begin()+i+1); // remove old states
//             break;
//          }
//       }

//       // step 2: update
//       // compute measurement residual (innovation)
//       auto y = ZMeasFromGPS_.z - (HfromGPS_ * statePred_.x);

//       // Innovation covariance
//       auto S = (HfromGPS_ * statePred_.P * HfromGPS_.transpose()) + RfromGPS_;

//       // optimal Kalman gain
//       auto K = statePred_.P * HfromGPS_.transpose() * S.inverse();

//       // Updated state estimate and it's covariacne
//       statePred_.x = statePred_.x + (K*y);
//       statePred_.P = statePred_.P - (K*HfromGPS_*statePred_.P);

//       ZLastMeasFromGPS_.time_stamp = ZMeasFromGPS_.time_stamp;
//       ZLastMeasFromGPS_.z = ZMeasFromGPS_.z;

//       // step 3: predict
//       if (!predict()) {
//          ROS_WARN("GPS Precition");
//          return;
//       }
//    }
// }

void LinearKF::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
   Vector3d acc;
   acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
   if(acc.norm() > 10000.0) {
      ROS_ERROR("[KF IMUCallback] Infinite measurement value. Ignoring measurement.");
      return;
   }
   float g = 9.80655f;

   if (bKFIsInitialized_)
   {  
      // step 1: measure
      ZMeasFromIMU_.time_stamp = msg->header.stamp;
      ZMeasFromIMU_.z(0) = msg->linear_acceleration.x;
      ZMeasFromIMU_.z(1) = msg->linear_acceleration.y;
      ZMeasFromIMU_.z(2) = msg->linear_acceleration.z - g;
      assert( ZMeasFromIMU_.time_stamp.toSec() >= ZLastMeasFromIMU_.time_stamp.toSec() );

      // // prevent the time confusing due to calculation delay
      // for (int i=0; i<statePredBuffer_.size(); i++)
      // {
      //    auto deltaT = ZMeasFromIMU_.time_stamp.toSec() - statePredBuffer_[i].time_stamp.toSec();
      //    if ((deltaT <= dt_) && (deltaT >= 0))
      //    {
      //       statePred_.time_stamp = statePredBuffer_[i].time_stamp;
      //       statePred_.x = statePredBuffer_[i].x;
      //       statePred_.P = statePredBuffer_[i].P;
      //       statePredBuffer_.erase(statePredBuffer_.begin(),statePredBuffer_.begin()+i+1); // remove old states
      //       break;
      //    }
      // }

      // // step 2: update
      // // compute measurement residual (innovation)
      // auto y = ZMeasFromIMU_.z - (HfromIMU_ * statePred_.x);

      // // Innovation covariance
      // auto S = (HfromIMU_ * statePred_.P * HfromIMU_.transpose()) + RfromIMU_;

      // // optimal Kalman gain
      // auto K = statePred_.P * HfromIMU_.transpose() * S.inverse();

      // // Updated state estimate and it's covariacne
      // statePred_.x = statePred_.x + (K*y);
      // statePred_.P = statePred_.P - (K*HfromIMU_*statePred_.P);

      // ZLastMeasFromIMU_.time_stamp = ZMeasFromIMU_.time_stamp;
      // ZLastMeasFromIMU_.z = ZMeasFromIMU_.z;

      // // step 3: predict
      // if (!predict()) {
      //    ROS_WARN("IMU Precition");
      //    return;
      // }
   }
}

void LinearKF::updateStatePredBuffer()
{
   KFState stateToStore;
   stateToStore.time_stamp = statePred_.time_stamp;
   stateToStore.x = statePred_.x;
   stateToStore.P = statePred_.P;
   statePredBuffer_.push_back(stateToStore);
   if (statePredBuffer_.size() > statePredBufferSize_) {
      statePredBuffer_.erase(statePredBuffer_.begin()); // remove first element in the buffer
   }
}

bool LinearKF::predict()
{
   // output: X_{k+1|k} & P_{k+1|k}
   assert( statePred_.x.size() == 18 );
   statePred_.x = F_ * statePred_.x;

   if (statePred_.x.norm() > 10000.0) {
      ROS_ERROR("state prediction exploded!!!");
      ROS_ERROR("Go to 0|0 Step");
      bKFIsInitialized_ = false;
      initKF();
      return false;
   }
   
   statePred_.P = F_ * statePred_.P * F_.transpose() + Q_;
   statePred_.time_stamp = ros::Time::now();  // means that k step of X_{k+1|k}

   // Publish State Estimate 
   publishState();

   // Add(store) state to buffer
   updateStatePredBuffer();

   return true;
}

void LinearKF::publishState()
{
   nav_msgs::Odometry stateMsg_;
   
   stateMsg_.header.frame_id = bodyFrame_;
   stateMsg_.header.stamp = statePred_.time_stamp;

   stateMsg_.pose.pose.position.x = statePred_.x(9,0) - statePred_.x(0,0);
   stateMsg_.pose.pose.position.y = statePred_.x(10,0) - statePred_.x(1,0);
   stateMsg_.pose.pose.position.z = statePred_.x(11,0) - statePred_.x(2,0);
   // ROS_WARN("UAV Pose: [%.3f, %.3f, %.3f]", statePred_.x(0,0), statePred_.x(1,0), statePred_.x(2,0));
   ROS_WARN("UGV Pose: [%.3f, %.3f, %.3f]", statePred_.x(9,0), statePred_.x(10,0), statePred_.x(12,0));

   stateMsg_.twist.twist.linear.x = statePred_.x(12,0);
   stateMsg_.twist.twist.linear.y = statePred_.x(13,0);
   stateMsg_.twist.twist.linear.z = statePred_.x(14,0);

   auto pxx = statePred_.P(0,0); auto pyy = statePred_.P(1,1); auto pzz = statePred_.P(2,2);
   auto vxx = statePred_.P(9,9); auto vyy = statePred_.P(10,10); auto vzz = statePred_.P(11,11);
   ROS_WARN("UAV Cov: [%.3f, %.3f, %.3f]", pxx, pyy, pzz);
   ROS_WARN("UGV Cov: [%.3f, %.3f, %.3f]", vxx, vyy, vzz);
   stateMsg_.pose.covariance[0] = pxx;
   stateMsg_.pose.covariance[7] = pyy; 
   stateMsg_.pose.covariance[14] = pzz;
   stateMsg_.twist.covariance[0] = vxx;
   stateMsg_.twist.covariance[7] = vyy; 
   stateMsg_.twist.covariance[14] = vzz; 

   statePub_.publish(stateMsg_);
}

// void LinearKF::compareCurrentAndOldTagTime(const ros::TimerEvent& event)
// {
//    auto deltaT = ros::Time::now().toSec() - ZMeasFromAprilTag_.time_stamp.toSec();
//    if (deltaT > maxMeasurementOffTime_)
//    {
//       ROS_WARN("AprilTag measurement is too old");
//       ROS_WARN("do KF initializing");
//       initKF();
//    }
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kf_node");
    ros::NodeHandle nh("~");

    std::unique_ptr<LinearKF> linear_kf_obj = std::make_unique<LinearKF>(&nh);

    ros::spin();
    return 0;
}
