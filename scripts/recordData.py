#! /usr/bin/env python

import os
import time
import threading
import math
import numpy as np
import pandas as pd

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class RecordData:
    
    def __init__(self):
        
        self.UAVPose_ = []
        self.UAVVel_ = []

        self.UAVLists_ = []
        self.IMULists_ = []
        self.aprilTagList_ = []

        self.KF0List_ = []
        self.KF0UncertaintyList_ = []
        self.KF1List_ = []
        self.KF1UncertaintyList_ = []

        self.GTList_ = []
        self.VelSpList_ = []

        rospy.Subscriber("/mavros/global_position/local", Odometry, self.UAVPoseCb)
        rospy.Subscriber("/mavros/imu/data", Imu, self.IMUCb)
        rospy.Subscriber("/magpie/perception/relative_pose", Odometry, self.aprilTagCb)

        rospy.Subscriber("/magpie/estimator/state0", Odometry, self.KF0Cb)
        rospy.Subscriber("/magpie/estimator/state1", Odometry, self.KF1Cb)

        rospy.Subscriber("/odom", Odometry, self.GTCb)
        rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, self.VelCb)
        
        rospy.on_shutdown(self.saveDataOnShutdown)

        rospy.loginfo("RecordData Node is initialized...");


    def UAVPoseCb(self, msg):
        
        time = msg.header.stamp.to_sec()
        self.UAVPose_ = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.UAVVel_ = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self.UAVLists_.append([time] + self.UAVPose_ + list(self.UAVVel_))

    def IMUCb(self, msg):

        acc = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.IMULists_.append(list(acc))

    def aprilTagCb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x + self.UAVPose_[0], msg.pose.pose.position.y + self.UAVPose_[1], msg.pose.pose.position.z + self.UAVPose_[2])
            velocity = (msg.twist.twist.linear.x + self.UAVVel_[0], msg.twist.twist.linear.y + self.UAVVel_[1], msg.twist.twist.linear.z + self.UAVVel_[2])
            self.aprilTagList_.append([time] + list(position) + list(velocity))
        
    def KF0Cb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x + self.UAVPose_[0], msg.pose.pose.position.y + self.UAVPose_[1], msg.pose.pose.position.z + self.UAVPose_[2])
            velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
            UAV_uncertainty = (msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[2])
            UGV_uncertainty = (msg.pose.covariance[3], msg.pose.covariance[4], msg.pose.covariance[5])
            UGV_vel_uncertainty = (msg.twist.covariance[0], msg.twist.covariance[1], msg.twist.covariance[2])
            self.KF0List_.append([time] + list(position) + list(velocity))
            self.KF0UncertaintyList_.append([time] + list(UAV_uncertainty) + list(UGV_uncertainty) + list(UGV_vel_uncertainty))

    def KF1Cb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x + self.UAVPose_[0], msg.pose.pose.position.y + self.UAVPose_[1], msg.pose.pose.position.z + self.UAVPose_[2])
            velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
            UAV_uncertainty = (msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[2])
            UGV_uncertainty = (msg.pose.covariance[3], msg.pose.covariance[4], msg.pose.covariance[5])
            UGV_vel_uncertainty = (msg.twist.covariance[0], msg.twist.covariance[1], msg.twist.covariance[2])
            self.KF1List_.append([time] + list(position) + list(velocity))
            self.KF1UncertaintyList_.append([time] + list(UAV_uncertainty) + list(UGV_uncertainty) + list(UGV_vel_uncertainty))
            
    def GTCb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
            self.GTList_.append([time] + list(position) + list(velocity))

    def VelCb(self, msg):

        time = rospy.Time.now().to_sec()
        vel = (msg.linear.x, msg.linear.y, msg.linear.z)
        self.VelSpList_.append([time] + list(vel))
    
    
    def saveDataOnShutdown(self):

        runWD = os.getcwd()
        cwd = runWD.split("/")[-1]
        assert( cwd == "autonomous_landing_ws" )
        
        GPScols = ['time','x','y','z','vx','vy','vz']
        IMUcols3 = ['ax','ay','az'] 
        cols = ['time', 'x', 'y', 'z', 'vx', 'vy', 'vz']
        uncerCols = ['time' ,'pxx', 'pyy', 'pzz', 'paxx', 'payy', 'pazz', 'pa_vxx', 'pa_vyy', 'pa_vzz']
        vCols = ['time', 'vx', 'vy', 'vz']
         
        GPSIdx = range(len(self.UAVLists_))
        dfGPSData = pd.DataFrame(self.UAVLists_, columns=GPScols, index=GPSIdx)
        IMUIdx = range(len(self.IMULists_))
        dfIMUData = pd.DataFrame(self.IMULists_, columns=IMUcols3, index=IMUIdx)
        
        GTIdx = range(len(self.GTList_))
        dfGTData = pd.DataFrame(self.GTList_, columns=cols, index=GTIdx)
        
        KF0Idx = range(len(self.KF0List_))
        dfKF0Data = pd.DataFrame(self.KF0List_, columns=cols, index=KF0Idx)
        KF0Idx = range(len(self.KF0UncertaintyList_))
        dfKF0UncerData = pd.DataFrame(self.KF0UncertaintyList_, columns=uncerCols, index=KF0Idx)
        
        KF1Idx = range(len(self.KF1List_))
        dfKF1Data = pd.DataFrame(self.KF1List_, columns=cols, index=KF1Idx)
        KF1Idx = range(len(self.KF1UncertaintyList_))
        dfKF1UncerData = pd.DataFrame(self.KF1UncertaintyList_, columns=uncerCols, index=KF1Idx)
        
        aprilTagIdx = range(len(self.aprilTagList_))
        dfAprilTagData = pd.DataFrame(self.aprilTagList_, columns=cols, index=aprilTagIdx)

        dfVelSpIdx = range(len(self.VelSpList_))
        dfVelSpData = pd.DataFrame(self.VelSpList_, columns=vCols, index=dfVelSpIdx)


        os.chdir("../data")
        dfAprilTagData.to_csv("aprilTagData.csv", index=False)
        dfKF0Data.to_csv("KF0Data.csv", index=False)
        dfKF1Data.to_csv("KF1Data.csv", index=False)
        dfGTData.to_csv("GTData.csv", index=False)

        dfGPSData.to_csv("GPSData.csv", index=False)
        dfIMUData.to_csv("IMUData.csv", index=False)

        dfKF0UncerData.to_csv("KF0UncertaintyData.csv", index=False)
        dfKF1UncerData.to_csv("KF1UncertaintyData.csv", index=False)
        
        dfVelSpData.to_csv("VelSpData.csv", index=False)

        rospy.loginfo("Data saved to CSV files on shutdown.")


def main():
    rospy.init_node("record_node", anonymous=True)
    recordDataObj = RecordData()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
