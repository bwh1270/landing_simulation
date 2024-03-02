#include "landing_simulation/LinearKalmanFilter.hpp"


LinearKF::LinearKF(ros::NodeHandle *nh) :
statePredBufferSize_(40),
bAprilTagIsBig_(true)
{
    // Get Parameters
    nh->getParam("body_frame", bodyFrame_);
    nh->getParam("big_aprilTag_frame", bigAprilTagFrame_);
    nh->getParam("small_aprilTag_frame", smallAprilTagFrame_);
    nh->getParam("height_of_ugv", UGVHeight_);
    assert( bodyFrame_ == "base_link" );

    nh->getParam("min_dt", dt_);
    nh->getParam("qu", qu_);
    nh->getParam("qa", qa_);
    nh->getParam("ru_pxy", ru_pxy_);
    nh->getParam("ru_pz", ru_pz_);
    nh->getParam("ru_vxy", ru_vxy_);
    nh->getParam("ru_vz", ru_vz_);
    nh->getParam("ru_a", ru_a_);
    nh->getParam("ra/u_pxy", rau_pxy_);
    nh->getParam("ra/u_pz", rau_pz_);
    nh->getParam("ra/u_vxy", rau_vxy_);
    nh->getParam("ra/u_vz", rau_vz_);

    nh->getParam("max_measurement_off_time", maxMeasurementOffTime_);
    tagTimeStamp_ = 0.0f;

    // Input & Output
    aprilTagSub_ = nh->subscribe("/magpie/perception/relative_pose", 1, &LinearKF::aprilTagCb, this);
    statePub_    = nh->advertise<nav_msgs::Odometry>("/magpie/estimator/state", 5);
    statePub2_   = nh->advertise<nav_msgs::Odometry>("/magpie/estimator/state2", 5);
    statePub3_   = nh->advertise<nav_msgs::Odometry>("/magpie/estimator/state3", 5);

    // Initialize subscribers (queue size 1 for minimum latency)
    GPSsub_.subscribe(*nh, "/mavros/global_position/local", 1);
    IMUsub_.subscribe(*nh, "/mavros/imu/data", 1);

    // Initialize synchronizer
    sync_.reset(new message_filters::Synchronizer<mySyncPolicy_>(mySyncPolicy_(10), GPSsub_, IMUsub_));
    sync_->registerCallback(boost::bind(&LinearKF::GPSIMUCb, this, _1, _2));
    

    ROS_INFO("Linear Kalman Filter Node is activated...");
    ROS_INFO("State buffer length corresponds to %.3f seconds", dt_*(double)statePredBufferSize_);

    initKF();

    // Define timer for constant loop rate
    predictTimer_ = nh->createTimer(ros::Duration(dt_), &LinearKF::predictLoop, this);
}

LinearKF::~LinearKF()
{
}

/** @brief - This model is for constant acceleration */
void LinearKF::setF(MatrixXd* F, double dt)
{
    F->resize(15,15);
    *F = MatrixXd::Identity(15,15);

    for (int i=0; i<3; i++) {
        (*F)(i,i+3) = dt;
        (*F)(i,i+6) = (dt*dt)/2.0;
        (*F)(i+3,i+6) = dt;
    }

    for (int i=0; i<3; i++) {
        (*F)(i+9,i+12) = dt;
    }
}

void LinearKF::setQ(MatrixXd* Q, double dt)
{
    Q->resize(15,15);
    *Q = MatrixXd::Zero(15,15);

    // set Qu
    for (int i=0; i<3; i++) {
        (*Q)(i,i)     = (qu_)*(dt*dt*dt*dt)/4.;  // p with p
        (*Q)(i+3,i+3) = (qu_)*(dt*dt);             // v with v
        (*Q)(i+6,i+6) = 1.0f;                        // a with a

        (*Q)(i,i+3)   = (qu_)*(dt*dt*dt)/2.0;      // p with v
        (*Q)(i,i+6)   = (qu_)*(dt*dt)/2.0;          // p with a

        (*Q)(i+3,i)   = (qu_)*(dt*dt*dt)/2.0;      // v with p
        (*Q)(i+3,i+6) = (qu_)*(dt);                  // v with a

        (*Q)(i+6,i)   = (qu_)*(dt*dt)/2.0;          // a with p
        (*Q)(i+6,i+3) = (qu_)*(dt);                  // a with v
    }

    // set Qa
    for (int i=0; i<3; i++) {
        (*Q)(i+9,i+9)   = (qa_)*(dt*dt*dt*dt)/4.; // x with x 
        (*Q)(i+12,i+12) = (qa_)*(dt*dt);            // v with v

        (*Q)(i+9,i+12)  = (qa_)*(dt*dt*dt)/2.0;    // x with v
        (*Q)(i+12,i+9)  = (qa_)*(dt*dt*dt)/2.0;    // v with x
    }
}

void LinearKF::setQc(MatrixXd* Qc, double dt)
{
    Qc->resize(15,15);
    *Qc = MatrixXd::Zero(15,15);

    // set Qc,u
    for (int i=0; i<3; i++) {
        (*Qc)(i,i)     = (qu_)*(dt*dt*dt*dt*dt)/20.;  // p with p
        (*Qc)(i+3,i+3) = (qu_)*(dt*dt*dt)/3.;           // v with v
        (*Qc)(i+6,i+6) = (qu_)*(dt);                      // a with a

        (*Qc)(i,i+3)   = (qu_)*(dt*dt*dt*dt)/8.0;      // p with v
        (*Qc)(i,i+6)   = (qu_)*(dt*dt*dt)/6.0;          // p with a

        (*Qc)(i+3,i)   = (qu_)*(dt*dt*dt*dt)/8.0;      // v with p
        (*Qc)(i+3,i+6) = (qu_)*(dt*dt)/2.0;              // v with a

        (*Qc)(i+6,i)   = (qu_)*(dt*dt*dt)/6.0;          // a with p
        (*Qc)(i+6,i+3) = (qu_)*(dt*dt)/2.0;              // a with v
    }

    // set Qc,a
    for (int i=0; i<3; i++) {
        (*Qc)(i+9,i+9)   = (qa_)*(dt*dt*dt*dt*dt)/20.; // x with x 
        (*Qc)(i+12,i+12) = (qa_)*(dt*dt*dt)/3.0;         // v with v

        (*Qc)(i+9,i+12)  = (qa_)*(dt*dt*dt*dt)/8.0;     // x with v
        (*Qc)(i+12,i+9)  = (qa_)*(dt*dt*dt*dt)/8.0;     // v with x
    }
}

void LinearKF::setH()
{
    H_.resize(15,15);
    H_ = MatrixXd::Zero(15,15);

    for (int i=0; i<9; i++) {
        H_(i,i) = 1.0;
    }
    for (int i=0; i<3; i++) {
        H_(i+9,i)   = -1.0;
        H_(i+9,i+9) = 1.0;
        H_(i+12,i+3) = -1.0;
        H_(i+12,i+12) = 1.0;
    }
}

void LinearKF::setR()
{
    R_.resize(15,15);
    R_ = MatrixXd::Identity(15,15);

    for (int i=0; i<2; i++) {
        R_(i,i)     = ru_pxy_;
        R_(i+3,i+3) = ru_vxy_;
        R_(i+9,i+9) = rau_pxy_;
        R_(i+12,i+12) = rau_vxy_;
    }
    for (int i=0; i<3; i++) {
        R_(i+6,i+6) = ru_a_;
    }
    R_(2,2)   = ru_pz_;
    R_(5,5)   = ru_vz_;
    R_(11,11) = rau_pz_;
    R_(14,14) = rau_vz_;
    // for (int i=0; i<3; i++) {
    //     R_(i,i)     = ru_p_;
    //     R_(i+3,i+3) = ru_v_;
    //     R_(i+6,i+6) = ru_a_;
    //     R_(i+9,i+9) = rau_p_;
    // }
}

void LinearKF::initKF()
{
    // Initialize state transition matrix and process noise covariance
    setF(&F_, 0.01);          
    setF(&F2_, 0.05);      
    setF(&F3_, 0.02);
    setQ(&Q_, 0.01);
    setQ(&Q2_, 0.05);
    setQ(&Q3_, 0.02);    
    setQc(&Qc_, 0.05);
    setH();
    setR();

    // initial state estimate (0|0 step)
    statePred_.time_stamp = ros::Time::now();
    statePred2_.time_stamp = ros::Time::now();
    statePred3_.time_stamp = ros::Time::now();
    statePred_.x = MatrixXd::Zero(15,1);
    statePred_.P = 500*MatrixXd::Identity(15,15);
    statePred2_.x = MatrixXd::Zero(15,1);
    statePred2_.P = 500*MatrixXd::Identity(15,15);
    statePred3_.x = MatrixXd::Zero(15,1);
    statePred3_.P = 500*MatrixXd::Identity(15,15);
    

    // statePred_.x(2,0)   = 3.0;        // z of UAV
    // statePred_.x(11,0)  = UGVHeight_; // z of UGV
    // statePred2_.x(2,0)  = 3.0;
    // statePred2_.x(11,0) = UGVHeight_;
    // statePred3_.x(2,0)  = 3.0;
    // statePred3_.x(11,0) = UGVHeight_;
    // statePred_.P(2,2)    = 0.0;
    // statePred_.P(11,11)  = 0.0;
    // statePred2_.P(2,2)   = 0.0;
    // statePred2_.P(11,11) = 0.0;
    // statePred3_.P(2,2)   = 0.0;
    // statePred3_.P(11,11) = 0.0; 

    stateForControl_ = statePred2_;
    statePredBuffer_.clear();  // Clear state buffer
    statePredBuffer2_.clear();
    statePredBuffer3_.clear();

    // initial three measurement vectors (0|0 step)
    // ZMeas_.z.resize(15,1); //z
    // ZMeas_.z = MatrixXd::Zero(15,1); //z
    ZMeas2_.z.resize(15,1);
    ZMeas2_.z = MatrixXd::Zero(15,1);       

    // First Prediction for X_{1|0} with X_{0|0}
    predict();
    predict2();
    predict3();
    bKFIsInitialized_ = true;

    ROS_INFO("Linear Kalman Filter is initialized.");
}


void LinearKF::aprilTagCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    Vector3d pos;
    Vector3d vel;
    pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    if ((pos.norm() > 10000.0) || (vel.norm() > 10000.0)) {
        ROS_ERROR("[KF aprilTagCb] Infinite measurement value. Ignoring measurement.");
        ROS_ERROR("Pos: [%.2f, %.2f, %.2f]", pos(0), pos(1), pos(2));
        ROS_ERROR("Vel: [%.2f, %.2f, %.2f]", vel(0), vel(1), vel(2));
        return;
    }

    // step 1: measure
    // ZMeas_.time_stamp = msg->header.stamp; //z
    // ZMeas_.z(9,0)  = msg->pose.pose.position.x;
    // ZMeas_.z(10,0) = msg->pose.pose.position.y;
    // ZMeas_.z(11,0) = msg->pose.pose.position.z;
    // ZMeas_.z(12,0) = msg->twist.twist.linear.x;
    // ZMeas_.z(13,0) = msg->twist.twist.linear.y;
    // ZMeas_.z(14,0) = msg->twist.twist.linear.z;
    // assert( ZMeas_.time_stamp.toSec() >= ZLastMeas_.time_stamp.toSec() ); //z
    ZMeas2_.time_stamp = ros::Time::now(); //msg->header.stamp;
    ZMeas2_.z(9,0)  = msg->pose.pose.position.x;
    ZMeas2_.z(10,0) = msg->pose.pose.position.y;
    ZMeas2_.z(11,0) = msg->pose.pose.position.z;
    ZMeas2_.z(12,0) = msg->twist.twist.linear.x;
    ZMeas2_.z(13,0) = msg->twist.twist.linear.y;
    ZMeas2_.z(14,0) = msg->twist.twist.linear.z;
    assert( ZMeas2_.time_stamp.toSec() >= ZLastMeas2_.time_stamp.toSec() );

    // prevent the time confusing due to calculation delay
    for (int i=0; i<statePredBuffer_.size(); i++)
    {
        auto deltaT = ZMeas2_.time_stamp.toSec() - statePredBuffer_[i].time_stamp.toSec(); //z
        if ((deltaT <= dt_) && (deltaT >= 0))
        {
        statePred_.time_stamp = statePredBuffer_[i].time_stamp;
        statePred_.x = statePredBuffer_[i].x;
        statePred_.P = statePredBuffer_[i].P;
        statePredBuffer_.erase(statePredBuffer_.begin(),statePredBuffer_.begin()+i+1); // remove old states
        break;
        }
    }
    for (int i=0; i<statePredBuffer2_.size(); i++)
    {
        auto deltaT = ZMeas2_.time_stamp.toSec() - statePredBuffer2_[i].time_stamp.toSec(); //z
        if ((deltaT <= dt_) && (deltaT >= 0))
        {
        statePred2_.time_stamp = statePredBuffer2_[i].time_stamp;
        statePred2_.x = statePredBuffer2_[i].x;
        statePred2_.P = statePredBuffer2_[i].P;
        statePredBuffer2_.erase(statePredBuffer2_.begin(),statePredBuffer2_.begin()+i+1); // remove old states
        break;
        }
    }

    // step 2: update
    // compute measurement residual (innovation)
    auto y = ZMeas2_.z - (H_ * statePred_.x); //z
    auto y2 = ZMeas2_.z - (H_ * statePred2_.x); //z

    // Innovation covariance
    auto S = (H_ * statePred_.P * H_.transpose()) + R_;
    auto S2 = (H_ * statePred2_.P * H_.transpose()) + R_;

    // optimal Kalman gain
    auto K = statePred_.P * H_.transpose() * S.inverse();
    auto K2 = statePred2_.P * H_.transpose() * S2.inverse();
    // cout << ">> K: " << endl;
    // cout << K << endl;

    // Updated state estimate and it's covariacne
    statePred_.x = statePred_.x + (K*y);
    statePred_.P = statePred_.P - (K*H_*statePred_.P);
    statePred2_.x = statePred2_.x + (K2*y2);
    statePred2_.P = statePred2_.P - (K2*H_*statePred2_.P);
    // cout << ">> P: " << endl;
    // cout << statePred_.P << endl;

    tagTimeStamp_  = ros::Time::now().toSec(); //msg->header.stamp.toSec();
    // ZLastMeas_.time_stamp = ZMeas_.time_stamp; //z
    // ZLastMeas_.z = ZMeas_.z; //z
    ZLastMeas2_.time_stamp = ZMeas2_.time_stamp;
    ZLastMeas2_.z = ZMeas2_.z;

    // step 3: predict
    if (!predict()) {
        return;
    }
    if (!predict2()) {
        return;
    }
}


void LinearKF::GPSIMUCb(const nav_msgs::Odometry::ConstPtr& gps, const sensor_msgs::Imu::ConstPtr& imu)
{
    Vector3d pos, vel, acc;
    pos << gps->pose.pose.position.x, gps->pose.pose.position.y, gps->pose.pose.position.z;
    vel << gps->twist.twist.linear.x, gps->twist.twist.linear.y, gps->twist.twist.linear.z;
    acc << imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z;
    if ((pos.norm() > 10000.0) || (vel.norm() > 10000.0) || (acc.norm() > 10000.0)) {
        ROS_ERROR("[KF GPSIMUCb] Infinite measurement value. Ignoring measurement.");
        return;
    }
    
    float g = 9.80655;

    /* KF & KF2 */
    // step 1: measure
    // ZMeas_.time_stamp = gps->header.stamp; //z
    // ZMeas_.z(0) = gps->pose.pose.position.x;
    // ZMeas_.z(1) = gps->pose.pose.position.y;
    // ZMeas_.z(2) = gps->pose.pose.position.z;
    // ZMeas_.z(3) = gps->twist.twist.linear.x;
    // ZMeas_.z(4) = gps->twist.twist.linear.y;
    // ZMeas_.z(5) = gps->twist.twist.linear.z;
    // ZMeas_.time_stamp = imu->header.stamp;
    // ZMeas_.z(6) = imu->linear_acceleration.x;
    // ZMeas_.z(7) = imu->linear_acceleration.y;
    // ZMeas_.z(8) = imu->linear_acceleration.z - g;
    // assert( ZMeas_.time_stamp.toSec() >= ZLastMeas_.time_stamp.toSec() ); //z
    
    
    /* KF3 */
    // compare Current And Old AprilTag Time
    auto deltaT = ros::Time::now().toSec() - tagTimeStamp_;
    if (deltaT > maxMeasurementOffTime_)
    {
        ROS_WARN("AprilTag measurement is too old");
        ROS_WARN("do KF initializing");
        initKF();
        return;
    }

    if (bKFIsInitialized_) {

        // step 1: measure
        ZMeas2_.time_stamp = ros::Time::now(); //gps->header.stamp;
        ZMeas2_.z(0) = gps->pose.pose.position.x;
        ZMeas2_.z(1) = gps->pose.pose.position.y;
        ZMeas2_.z(2) = gps->pose.pose.position.z;
        ZMeas2_.z(3) = gps->twist.twist.linear.x;
        ZMeas2_.z(4) = gps->twist.twist.linear.y;
        ZMeas2_.z(5) = gps->twist.twist.linear.z;
        ZMeas2_.z(6) = imu->linear_acceleration.x;
        ZMeas2_.z(7) = imu->linear_acceleration.y;
        ZMeas2_.z(8) = imu->linear_acceleration.z - g;

        for (int i=0; i<statePredBuffer3_.size(); i++)
        {
            auto deltaT = ZMeas2_.time_stamp.toSec() - statePredBuffer3_[i].time_stamp.toSec();
            if ((deltaT <= 0.02) && (deltaT >= 0))
            {
            statePred3_.time_stamp = statePredBuffer3_[i].time_stamp;
            statePred3_.x = statePredBuffer3_[i].x;
            statePred3_.P = statePredBuffer3_[i].P;
            statePredBuffer3_.erase(statePredBuffer3_.begin(),statePredBuffer3_.begin()+i+1); // remove old states
            break;
            }
        }

        // step 2: update
        // compute measurement residual (innovation)
        auto y = ZMeas2_.z - (H_ * statePred3_.x);

        // Innovation covariance
        auto S = (H_ * statePred3_.P * H_.transpose()) + R_;

        // optimal Kalman gain
        auto K = statePred3_.P * H_.transpose() * S.inverse();

        // Updated state estimate and it's covariacne
        statePred3_.x = statePred3_.x + (K*y);
        statePred3_.P = statePred3_.P - (K*H_*statePred3_.P);

        ZLastMeas2_.time_stamp = ZMeas2_.time_stamp;
        ZLastMeas2_.z = ZMeas2_.z;

        // step 3: predict
        if (!predict3()) {
            return;
        }
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

void LinearKF::updateStatePredBuffer2()
{
   KFState stateToStore;
   stateToStore.time_stamp = statePred2_.time_stamp;
   stateToStore.x = statePred2_.x;
   stateToStore.P = statePred2_.P;
   statePredBuffer2_.push_back(stateToStore);
   if (statePredBuffer2_.size() > statePredBufferSize_) {
      statePredBuffer2_.erase(statePredBuffer2_.begin()); // remove first element in the buffer
   }
}

void LinearKF::updateStatePredBuffer3()
{
   KFState stateToStore;
   stateToStore.time_stamp = statePred3_.time_stamp;
   stateToStore.x = statePred3_.x;
   stateToStore.P = statePred3_.P;
   statePredBuffer3_.push_back(stateToStore);
   if (statePredBuffer3_.size() > statePredBufferSize_) {
      statePredBuffer3_.erase(statePredBuffer3_.begin()); // remove first element in the buffer
   }
}

bool LinearKF::predict()
{
    // output: X_{k+1|k} & P_{k+1|k}
    assert( statePred_.x.size() == 15 );
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

bool LinearKF::predict2()
{
    // output: X_{k+1|k} & P_{k+1|k}
    assert( statePred2_.x.size() == 15 );
    statePred2_.x = F2_ * statePred2_.x;

    if (statePred2_.x.norm() > 10000.0) {
        ROS_ERROR("state prediction exploded!!!");
        ROS_ERROR("Go to 0|0 Step");
        bKFIsInitialized_ = false;
        initKF();
        return false;
    }

    statePred2_.P = F2_ * statePred2_.P * F2_.transpose() + Q2_;
    statePred2_.time_stamp = ros::Time::now();  // means that k step of X_{k+1|k}

    // Publish State Estimate 
    stateForControl_ = statePred2_;
    publishState2();

    // Add(store) state to buffer
    updateStatePredBuffer2();

    return true;
}

bool LinearKF::predict3()
{
    // output: X_{k+1|k} & P_{k+1|k}
    assert( statePred3_.x.size() == 15 );
    statePred3_.x = F3_ * statePred3_.x;

    if (statePred3_.x.norm() > 10000.0) {
        ROS_ERROR("state prediction exploded!!!");
        ROS_ERROR("Go to 0|0 Step");
        bKFIsInitialized_ = false;
        initKF();
        return false;
    }

    statePred3_.P = F3_ * statePred3_.P * F3_.transpose() + Q3_;
    statePred3_.time_stamp = ros::Time::now();  // means that k step of X_{k+1|k}

    // Publish State Estimate 
    publishState3();

    // Add(store) state to buffer
    updateStatePredBuffer3();

    return true;
}

bool LinearKF::predictAsynchornously()
{
    // output: X_{k+1|k} & P_{k+1|k}
    assert( stateForControl_.x.size() == 15 );
    stateForControl_.x = F_ * stateForControl_.x;

    if (stateForControl_.x.norm() > 10000.0) {
        ROS_ERROR("state prediction exploded!!!");
        ROS_ERROR("Go to 0|0 Step");
        bKFIsInitialized_ = false;
        initKF();
        return false;
    }

    stateForControl_.P = F_ * stateForControl_.P * F_.transpose() + Q_;
    stateForControl_.time_stamp = ros::Time::now();  // means that k step of X_{k+1|k}

    // Publish State Estimate 
    publishState2();
    
    return true;
}

void LinearKF::publishState()
{
    nav_msgs::Odometry stateMsg_;

    stateMsg_.header.frame_id = bodyFrame_;
    stateMsg_.header.stamp = statePred_.time_stamp;

    stateMsg_.pose.pose.position.x = statePred_.x(9,0) - statePred_.x(0,0);  // relative pose (x)
    stateMsg_.pose.pose.position.y = statePred_.x(10,0) - statePred_.x(1,0); // relative pose (y)
    stateMsg_.pose.pose.position.z = statePred_.x(11,0) - statePred_.x(2,0); // relative pose (y)
    // ROS_INFO("UAV Pose: [%.3f, %.3f, %.3f]", statePred_.x(0,0), statePred_.x(1,0), statePred_.x(2,0));
    // ROS_INFO("UGV Pose: [%.3f, %.3f, %.3f]", statePred_.x(9,0), statePred_.x(10,0), statePred_.x(11,0));
    // ROS_INFO("UGV Vel:  [%.3f, %.3f, %.3f]", statePred_.x(12,0), statePred_.x(13,0), statePred_.x(14,0));

    stateMsg_.twist.twist.linear.x = statePred_.x(12,0);  // absolute vel (x of UGV)
    stateMsg_.twist.twist.linear.y = statePred_.x(13,0);  // absolute vel (y of UGV)
    stateMsg_.twist.twist.linear.z = statePred_.x(14,0);  // absolute vel (z of UGV)

    auto pxx = statePred_.P(0,0); auto pyy = statePred_.P(1,1); auto pzz = statePred_.P(2,2); // only UAV pose
    auto paxx = stateForControl_.P(9,9); auto payy = stateForControl_.P(10,10); auto pazz = stateForControl_.P(11,11); // only UGV pose
    auto vxx = statePred_.P(12,12); auto vyy = statePred_.P(13,13); auto vzz = statePred_.P(14,14);  // only UGV vel
    // ROS_INFO("UAV Cov: [%.3f, %.3f, %.3f]", pxx, pyy, pzz);
    // ROS_INFO("UGV Cov: [%.3f, %.3f, %.3f]", vxx, vyy, vzz);
    stateMsg_.pose.covariance[0]   = pxx;
    stateMsg_.pose.covariance[1]   = pyy; 
    stateMsg_.pose.covariance[2]   = pzz;
    stateMsg_.pose.covariance[3]   = paxx;
    stateMsg_.pose.covariance[4]   = payy;
    stateMsg_.pose.covariance[5]   = pazz;
    stateMsg_.twist.covariance[0]  = vxx;
    stateMsg_.twist.covariance[7]  = vyy;
    stateMsg_.twist.covariance[14] = vyy; 

    statePub_.publish(stateMsg_);
}

void LinearKF::publishState2()
{
    nav_msgs::Odometry stateMsg_;

    stateMsg_.header.frame_id = bodyFrame_;
    stateMsg_.header.stamp = stateForControl_.time_stamp;

    stateMsg_.pose.pose.position.x = stateForControl_.x(9,0) - stateForControl_.x(0,0);  // relative pose (x)
    stateMsg_.pose.pose.position.y = stateForControl_.x(10,0) - stateForControl_.x(1,0); // relative pose (y)
    stateMsg_.pose.pose.position.z = stateForControl_.x(11,0) - stateForControl_.x(2,0); // relative pose (y)
    // ROS_INFO("UAV Pose: [%.3f, %.3f, %.3f]", statePred_.x(0,0), statePred_.x(1,0), statePred_.x(2,0));
    // ROS_INFO("UGV Pose: [%.3f, %.3f, %.3f]", statePred_.x(9,0), statePred_.x(10,0), statePred_.x(11,0));
    // ROS_INFO("UGV Vel:  [%.3f, %.3f, %.3f]", statePred_.x(12,0), statePred_.x(13,0), statePred_.x(14,0));

    stateMsg_.twist.twist.linear.x = stateForControl_.x(12,0);  // absolute vel (x of UGV)
    stateMsg_.twist.twist.linear.y = stateForControl_.x(13,0);  // absolute vel (y of UGV)
    stateMsg_.twist.twist.linear.z = stateForControl_.x(14,0);  // absolute vel (z of UGV)

    auto pxx = stateForControl_.P(0,0); auto pyy = stateForControl_.P(1,1); auto pzz = stateForControl_.P(2,2); // only UAV pose
    auto paxx = stateForControl_.P(9,9); auto payy = stateForControl_.P(10,10); auto pazz = stateForControl_.P(11,11); // only UGV pose
    auto vxx = stateForControl_.P(12,12); auto vyy = stateForControl_.P(13,13); auto vzz = stateForControl_.P(14,14);  // only UGV vel
    // ROS_INFO("UAV Cov: [%.3f, %.3f, %.3f]", pxx, pyy, pzz);
    // ROS_INFO("UGV Cov: [%.3f, %.3f, %.3f]", vxx, vyy, vzz);
    stateMsg_.pose.covariance[0]   = pxx;
    stateMsg_.pose.covariance[1]   = pyy; 
    stateMsg_.pose.covariance[2]   = pzz;
    stateMsg_.pose.covariance[3]   = paxx;
    stateMsg_.pose.covariance[4]   = payy;
    stateMsg_.pose.covariance[5]   = pazz;
    stateMsg_.twist.covariance[0]  = vxx;
    stateMsg_.twist.covariance[7]  = vyy;
    stateMsg_.twist.covariance[14] = vyy; 

    statePub2_.publish(stateMsg_);
}

void LinearKF::publishState3()
{
    nav_msgs::Odometry stateMsg_;

    stateMsg_.header.frame_id = bodyFrame_;
    stateMsg_.header.stamp = statePred3_.time_stamp;

    stateMsg_.pose.pose.position.x = statePred3_.x(9,0) - statePred3_.x(0,0);  // relative pose (x)
    stateMsg_.pose.pose.position.y = statePred3_.x(10,0) - statePred3_.x(1,0); // relative pose (y)
    stateMsg_.pose.pose.position.z = statePred3_.x(11,0) - statePred3_.x(2,0); // relative pose (y)
    // ROS_INFO("UAV Pose: [%.3f, %.3f, %.3f]", statePred_.x(0,0), statePred_.x(1,0), statePred_.x(2,0));
    // ROS_INFO("UGV Pose: [%.3f, %.3f, %.3f]", statePred_.x(9,0), statePred_.x(10,0), statePred_.x(11,0));
    // ROS_INFO("UGV Vel:  [%.3f, %.3f, %.3f]", statePred_.x(12,0), statePred_.x(13,0), statePred_.x(14,0));

    stateMsg_.twist.twist.linear.x = statePred3_.x(12,0);  // absolute vel (x of UGV)
    stateMsg_.twist.twist.linear.y = statePred3_.x(13,0);  // absolute vel (y of UGV)
    stateMsg_.twist.twist.linear.z = statePred3_.x(14,0);  // absolute vel (z of UGV)

    auto pxx = statePred3_.P(0,0); auto pyy = statePred3_.P(1,1); auto pzz = statePred3_.P(2,2); // only UAV pose
    auto paxx = statePred3_.P(9,9); auto payy = statePred3_.P(10,10); auto pazz = statePred3_.P(11,11); // only UGV pose
    auto vxx = statePred3_.P(12,12); auto vyy = statePred3_.P(13,13); auto vzz = statePred3_.P(14,14);  // only UGV vel
    // ROS_INFO("UAV Cov: [%.3f, %.3f, %.3f]", pxx, pyy, pzz);
    // ROS_INFO("UGV Cov: [%.3f, %.3f, %.3f]", vxx, vyy, vzz);
    stateMsg_.pose.covariance[0]   = pxx;
    stateMsg_.pose.covariance[1]   = pyy; 
    stateMsg_.pose.covariance[2]   = pzz;
    stateMsg_.pose.covariance[3]   = paxx;
    stateMsg_.pose.covariance[4]   = payy;
    stateMsg_.pose.covariance[5]   = pazz;
    stateMsg_.twist.covariance[0]  = vxx;
    stateMsg_.twist.covariance[7]  = vyy;
    stateMsg_.twist.covariance[14] = vyy; 

    statePub3_.publish(stateMsg_);
}

void LinearKF::predictLoop(const ros::TimerEvent& event)
{
    // compare Current And Old AprilTag Time
    auto deltaT = ros::Time::now().toSec() - tagTimeStamp_;
    if (deltaT > maxMeasurementOffTime_)
    {
        ROS_WARN("AprilTag measurement is too old");
        ROS_WARN("do KF initializing");
        initKF();
    }

    if (bKFIsInitialized_) {
        if (deltaT > dt_) {
            // After update->predict, too fast predict means i use update->predict information little.
            predict();
            predictAsynchornously();
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kf_node");
    ros::NodeHandle nh("~");

    std::unique_ptr<LinearKF> linear_kf_obj = std::make_unique<LinearKF>(&nh);

    ros::spin();
    return 0;
}
