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

        self.UAVLists_ = []
        self.IMULists_ = []
        self.aprilTagList_ = []
        self.KFList_ = []
        self.KFUncertaintyList_ = []
        self.KF2List_ = []
        self.KF2UncertaintyList_ = []
        self.KF3List_ = []
        self.GTList_ = []
        self.VelSpList_ = []

        rospy.Subscriber("/mavros/global_position/local", Odometry, self.UAVPoseCb)
        rospy.Subscriber("/mavros/imu/data", Imu, self.IMUCb)
        rospy.Subscriber("/magpie/perception/relative_pose", Odometry, self.aprilTagCb)
        rospy.Subscriber("/magpie/estimator/state", Odometry, self.KFCb)
        rospy.Subscriber("/magpie/estimator/state2", Odometry, self.KF2Cb)
        rospy.Subscriber("/magpie/estimator/state3", Odometry, self.KF3Cb)
        rospy.Subscriber("/odom", Odometry, self.GTCb)
        rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, self.VelCb)
        
        rospy.on_shutdown(self.saveDataOnShutdown)

        rospy.loginfo("RecordData Node is initialized...");


    def UAVPoseCb(self, msg):
        
        time = msg.header.stamp.to_sec()
        self.UAVPose_ = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self.UAVLists_.append([time] + self.UAVPose_ + list(vel))

    def IMUCb(self, msg):

        acc = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.IMULists_.append(list(acc))

    def aprilTagCb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x + self.UAVPose_[0], msg.pose.pose.position.y + self.UAVPose_[1], msg.pose.pose.position.z + self.UAVPose_[2])
            self.aprilTagList_.append([time] + list(position))
        
    def KFCb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x + self.UAVPose_[0], msg.pose.pose.position.y + self.UAVPose_[1], msg.pose.pose.position.z + self.UAVPose_[2])
            uncertainty = (msg.pose.covariance[3], msg.pose.covariance[4], msg.pose.covariance[5])
            self.KFList_.append([time] + list(position))
            self.KFUncertaintyList_.append([time] + list(uncertainty))

    def KF2Cb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x + self.UAVPose_[0], msg.pose.pose.position.y + self.UAVPose_[1], msg.pose.pose.position.z + self.UAVPose_[2])
            uncertainty = (msg.pose.covariance[3], msg.pose.covariance[4], msg.pose.covariance[5])
            self.KF2List_.append([time] + list(position))
            self.KF2UncertaintyList_.append([time] + list(uncertainty))

    def KF3Cb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x + self.UAVPose_[0], msg.pose.pose.position.y + self.UAVPose_[1], msg.pose.pose.position.z + self.UAVPose_[2])
            self.KF3List_.append([time] + list(position))
            
    def GTCb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            self.GTList_.append([time] + list(position))

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
         
        GPSIdx = range(len(self.UAVLists_))
        dfGPSData = pd.DataFrame(self.UAVLists_, columns=GPScols, index=GPSIdx)
        IMUIdx = range(len(self.IMULists_))
        dfIMUData = pd.DataFrame(self.IMULists_, columns=IMUcols3, index=IMUIdx)
        
        
        cols = ['time', 'x', 'y', 'z']
        uncerCols = ['time' ,'pxx', 'pyy', 'pzz']
        GTIdx = range(len(self.GTList_))
        dfGTData = pd.DataFrame(self.GTList_, columns=cols, index=GTIdx)
        KFIdx = range(len(self.KFList_))
        dfKFData = pd.DataFrame(self.KFList_, columns=cols, index=KFIdx)
        KFIdx = range(len(self.KFUncertaintyList_))
        dfKFUncerData = pd.DataFrame(self.KFUncertaintyList_, columns=uncerCols, index=KFIdx)
        KF2Idx = range(len(self.KF2List_))
        dfKF2Data = pd.DataFrame(self.KF2List_, columns=cols, index=KF2Idx)
        KF2Idx = range(len(self.KF2UncertaintyList_))
        dfKF2UncerData = pd.DataFrame(self.KF2UncertaintyList_, columns=uncerCols, index=KF2Idx)
        KF3Idx = range(len(self.KF3List_))
        dfKF3Data = pd.DataFrame(self.KF3List_, columns=cols, index=KF3Idx)
        aprilTagIdx = range(len(self.aprilTagList_))
        dfAprilTagData = pd.DataFrame(self.aprilTagList_, columns=cols, index=aprilTagIdx)

        vCols = ['time', 'vx', 'vy', 'vz']
        dfVelSpIdx = range(len(self.VelSpList_))
        dfVelSpData = pd.DataFrame(self.VelSpList_, columns=vCols, index=dfVelSpIdx)

        os.chdir("../data")
        dfAprilTagData.to_csv("aprilTagData.csv", index=False)
        dfKFData.to_csv("KFData.csv", index=False)
        dfKF2Data.to_csv("KF2Data.csv", index=False)
        dfKF3Data.to_csv("KF3Data.csv", index=False)
        dfGTData.to_csv("GTData.csv", index=False)

        dfGPSData.to_csv("GPSData.csv", index=False)
        dfIMUData.to_csv("IMUData.csv", index=False)

        dfKFUncerData.to_csv("KFUncertaintyData.csv", index=False)
        dfKF2UncerData.to_csv("KF2UncertaintyData.csv", index=False)
        
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
    
