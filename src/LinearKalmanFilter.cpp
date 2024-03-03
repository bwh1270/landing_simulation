#include "landing_simulation/LinearKalmanFilter.hpp"


LinearKF::LinearKF(ros::NodeHandle *nh)
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
    bKFIsInitialized_ = false;
    bAprilTagIsBig_ = true;
    bAprilTagIsMeasured_ = false;

    // Input & Output
    aprilTagSub_ = nh->subscribe("/magpie/perception/relative_pose", 1, &LinearKF::aprilTagCb, this);
    // Initialize subscribers (queue size 1 for minimum latency)
    GPSsub_.subscribe(*nh, "/mavros/global_position/local", 1);
    IMUsub_.subscribe(*nh, "/mavros/imu/data", 1);

    // Initialize synchronizer
    sync_.reset(new message_filters::Synchronizer<mySyncPolicy_>(mySyncPolicy_(10), GPSsub_, IMUsub_));
    sync_->registerCallback(boost::bind(&LinearKF::GPSIMUCb, this, _1, _2));

    statePub0_ = nh->advertise<nav_msgs::Odometry>("/magpie/estimator/state0", 5);  // 0 means Low frequency
    statePub1_ = nh->advertise<nav_msgs::Odometry>("/magpie/estimator/state1", 5);  // 1 means High frequency

    initKF();
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
    setF(&F0_, 0.05);      
    setF(&F1_, 0.02);
    setQ(&Q0_, 0.05);
    setQ(&Q1_, 0.02);    
    setQc(&Qc_, 0.05);
    setH();
    setR();

    // initial state estimate (0|0 step)
    statePred0_.time_stamp = ros::Time::now();
    statePred0_.x = MatrixXd::Zero(15,1);
    statePred0_.P = 500*MatrixXd::Identity(15,15);
    statePred1_ = statePred0_;

    // First Prediction for X_{1|0} with X_{0|0}
    predict0();
    predict1();

    // initial three measurement vectors, just initializing no meaning of KF
    ZMeas_.time_stamp = ros::Time::now();
    ZMeas_.z.resize(15,1);
    ZMeas_.z = MatrixXd::Zero(15,1);

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

    /* KF w.r.t. Low freqeuncy measurement (AprilTag) */
    if (bKFIsInitialized_) {

        // step 1: measure
        ZMeas_.time_stamp = ros::Time::now(); // measuring time
        ZMeas_.z(9,0)  = msg->pose.pose.position.x;
        ZMeas_.z(10,0) = msg->pose.pose.position.y;
        ZMeas_.z(11,0) = msg->pose.pose.position.z;
        ZMeas_.z(12,0) = msg->twist.twist.linear.x;
        ZMeas_.z(13,0) = msg->twist.twist.linear.y;
        ZMeas_.z(14,0) = msg->twist.twist.linear.z;
        assert( ZMeas_.time_stamp.toSec() >= ZLastMeas_.time_stamp.toSec() );

        // prevent the new GPS&IMU measurement interrupting Apriltag update due to calculating delay
        auto ZMeasStatic = ZMeas_;

        // step 2: update
        // compute measurement residual (innovation)
        auto y = ZMeasStatic.z - (H_ * statePred0_.x); //z


        // Innovation covariance
        auto S = (H_ * statePred0_.P * H_.transpose()) + R_;

        // optimal Kalman gain
        auto K = statePred0_.P * H_.transpose() * S.inverse();
        // cout << ">> K: " << endl;
        // cout << K << endl;

        // Updated state estimate and it's covariacne
        statePred0_.x = statePred0_.x + (K*y);
        statePred0_.P = statePred0_.P - (K*H_*statePred0_.P);
        // cout << ">> P: " << endl;
        // cout << statePred_.P << endl;

        ZLastMeas_.time_stamp = ZMeasStatic.time_stamp;
        ZLastMeas_.z = ZMeasStatic.z;

        // step 3: predict
        if (!predict0()) {
            return;
        }
        tagTimeStamp_  = ros::Time::now().toSec(); // record measuring time
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
    
    float g = 9.80666;

    /* KF w.r.t. High freqeuncy measurement (GPS&IMU) */
    // Initialize the Kalman Filter when no AprilTag measurements detected in maxMeasurementOffTime_
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
        ZMeas_.time_stamp = ros::Time::now();
        ZMeas_.z(0) = gps->pose.pose.position.x;
        ZMeas_.z(1) = gps->pose.pose.position.y;
        ZMeas_.z(2) = gps->pose.pose.position.z;
        ZMeas_.z(3) = gps->twist.twist.linear.x;
        ZMeas_.z(4) = gps->twist.twist.linear.y;
        ZMeas_.z(5) = gps->twist.twist.linear.z;
        ZMeas_.z(6) = imu->linear_acceleration.x;
        ZMeas_.z(7) = imu->linear_acceleration.y;
        ZMeas_.z(8) = imu->linear_acceleration.z - g;
        assert( ZMeas_.time_stamp.toSec() >= ZLastMeas_.time_stamp.toSec() );

        // prevent the new GPS&IMU measurement interrupting Apriltag update due to calculating delay
        auto ZMeasStatic = ZMeas_;

        // step 2: update
        // compute measurement residual (innovation)
        auto y = ZMeasStatic.z - (H_ * statePred1_.x);

        // Innovation covariance
        auto S = (H_ * statePred1_.P * H_.transpose()) + R_;

        // optimal Kalman gain
        auto K = statePred1_.P * H_.transpose() * S.inverse();

        // Updated state estimate and it's covariacne
        statePred1_.x = statePred1_.x + (K*y);
        statePred1_.P = statePred1_.P - (K*H_*statePred1_.P);

        ZLastMeas_.time_stamp = ZMeasStatic.time_stamp;
        ZLastMeas_.z = ZMeasStatic.z;

        // step 3: predict
        if (!predict1()) {
            return;
        }
    }
}

bool LinearKF::predict0()
{
    // output: X_{k+1|k} & P_{k+1|k}
    assert( statePred0_.x.size() == 15 );
    statePred0_.x = F0_ * statePred0_.x;

    if (statePred0_.x.norm() > 10000.0) {
        ROS_ERROR("state prediction exploded!!!");
        ROS_ERROR("Go to 0|0 Step");
        bKFIsInitialized_ = false;
        initKF();
        return false;
    }

    statePred0_.P = F0_ * statePred0_.P * F0_.transpose() + Q0_;
    statePred0_.time_stamp = ros::Time::now();  // means that k step of X_{k+1|k}

    // Publish State Estimate 
    publishState0();

    return true;
}

bool LinearKF::predict1()
{
    // output: X_{k+1|k} & P_{k+1|k}
    assert( statePred1_.x.size() == 15 );
    statePred1_.x = F1_ * statePred1_.x;

    if (statePred1_.x.norm() > 10000.0) {
        ROS_ERROR("state prediction exploded!!!");
        ROS_ERROR("Go to 0|0 Step");
        bKFIsInitialized_ = false;
        initKF();
        return false;
    }

    statePred1_.P = F1_ * statePred1_.P * F1_.transpose() + Q1_;
    statePred1_.time_stamp = ros::Time::now();  // means that k step of X_{k+1|k}

    // Publish State Estimate 
    publishState1();

    return true;
}

void LinearKF::publishState0()
{
    nav_msgs::Odometry stateMsg_;

    stateMsg_.header.frame_id = bodyFrame_;
    stateMsg_.header.stamp = statePred0_.time_stamp;

    stateMsg_.pose.pose.position.x = statePred0_.x(9,0) - statePred0_.x(0,0);  // relative pose (x)
    stateMsg_.pose.pose.position.y = statePred0_.x(10,0) - statePred0_.x(1,0); // relative pose (y)
    stateMsg_.pose.pose.position.z = statePred0_.x(11,0) - statePred0_.x(2,0); // relative pose (y)

    stateMsg_.twist.twist.linear.x = statePred0_.x(12,0);  // absolute vel (x of UGV)
    stateMsg_.twist.twist.linear.y = statePred0_.x(13,0);  // absolute vel (y of UGV)
    stateMsg_.twist.twist.linear.z = statePred0_.x(14,0);  // absolute vel (z of UGV)

    auto pxx = statePred0_.P(0,0); auto pyy = statePred0_.P(1,1); auto pzz = statePred0_.P(2,2); // only UAV pose
    auto paxx = statePred0_.P(9,9); auto payy = statePred0_.P(10,10); auto pazz = statePred0_.P(11,11); // only UGV pose
    auto vxx = statePred0_.P(12,12); auto vyy = statePred0_.P(13,13); auto vzz = statePred0_.P(14,14);  // only UGV vel

    stateMsg_.pose.covariance[0]   = pxx;
    stateMsg_.pose.covariance[1]   = pyy; 
    stateMsg_.pose.covariance[2]   = pzz;
    stateMsg_.pose.covariance[3]   = paxx;
    stateMsg_.pose.covariance[4]   = payy;
    stateMsg_.pose.covariance[5]   = pazz;
    stateMsg_.twist.covariance[0] = vxx;
    stateMsg_.twist.covariance[1] = vyy;
    stateMsg_.twist.covariance[2] = vzz; 

    statePub0_.publish(stateMsg_);
}

void LinearKF::publishState1()
{
    nav_msgs::Odometry stateMsg_;

    stateMsg_.header.frame_id = bodyFrame_;
    stateMsg_.header.stamp = statePred1_.time_stamp;

    stateMsg_.pose.pose.position.x = statePred1_.x(9,0) - statePred1_.x(0,0);  // relative pose (x)
    stateMsg_.pose.pose.position.y = statePred1_.x(10,0) - statePred1_.x(1,0); // relative pose (y)
    stateMsg_.pose.pose.position.z = statePred1_.x(11,0) - statePred1_.x(2,0); // relative pose (y)
    stateMsg_.twist.twist.linear.x = statePred1_.x(12,0);  // absolute vel (x of UGV)
    stateMsg_.twist.twist.linear.y = statePred1_.x(13,0);  // absolute vel (y of UGV)
    stateMsg_.twist.twist.linear.z = statePred1_.x(14,0);  // absolute vel (z of UGV)

    auto pxx = statePred1_.P(0,0); auto pyy = statePred1_.P(1,1); auto pzz = statePred1_.P(2,2); // only UAV pose
    auto paxx = statePred1_.P(9,9); auto payy = statePred1_.P(10,10); auto pazz = statePred1_.P(11,11); // only UGV pose
    auto vxx = statePred1_.P(12,12); auto vyy = statePred1_.P(13,13); auto vzz = statePred1_.P(14,14);  // only UGV vel

    stateMsg_.pose.covariance[0]   = pxx;
    stateMsg_.pose.covariance[1]   = pyy; 
    stateMsg_.pose.covariance[2]   = pzz;
    stateMsg_.pose.covariance[3]   = paxx;
    stateMsg_.pose.covariance[4]   = payy;
    stateMsg_.pose.covariance[5]   = pazz;
    stateMsg_.twist.covariance[0]  = vxx;
    stateMsg_.twist.covariance[1]  = vyy;
    stateMsg_.twist.covariance[2] = vyy; 

    statePub1_.publish(stateMsg_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kf_node");
    ros::NodeHandle nh("~");

    std::unique_ptr<LinearKF> linear_kf_obj = std::make_unique<LinearKF>(&nh);

    ros::spin();
    return 0;
}
