#! /usr/bin/env python

import sys, argparse
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
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import AttitudeTarget


class RecordData:
    
    def __init__(self, aims="kalman_tuning"):

        self.aims = aims
        
        if (self.aims == "kalman_tuning"):

            self.GTList_ = []  # gt
            self.aprilTagList_ = []  # meas
            self.KF0List_ = [] # esti
            self.KF1List_ = [] # esti

            self.KF0UncertaintyList_ = []
            self.KF1UncertaintyList_ = []

            # to cal var
            self.UAVPose_ = []
            self.UAVVel_ = []
            self.UAVAtti_ = []
            self.UAVLists_ = []
            self.IMULists_ = [] 

            rospy.Subscriber("/odom", Odometry, self.GTCb)
            rospy.Subscriber("/magpie/perception/relative_pose", Odometry, self.aprilTagCb)
            rospy.Subscriber("/magpie/estimator/state0", Odometry, self.KF0Cb)
            rospy.Subscriber("/magpie/estimator/state1", Odometry, self.KF1Cb)

            rospy.Subscriber("/mavros/global_position/local", Odometry, self.UAVPoseCb)
            rospy.Subscriber("/mavros/imu/data", Imu, self.IMUCb)


        elif (self.aims == "velocity_tuning"):

            self.UAVPose_ = []
            self.UAVVel_ = []
            self.UAVAtti_ = []
            self.UAVLists_ = []
            self.VelSpList_ = []

            rospy.Subscriber("/mavros/global_position/local", Odometry, self.UAVPoseCb)
            rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, self.VelCb)
            

        elif (self.aims == "sliding_throttle"):
        
            self.UAVPose_ = []
            self.UAVVel_ = []
            self.UAVAtti_ = []
            self.UAVLists_ = []
            self.thrSpList_ = []
            self.IMULists_ = [] 

            rospy.Subscriber("/mavros/global_position/local", Odometry, self.UAVPoseCb)
            rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.thrSpCb)
            rospy.Subscriber("/mavros/imu/data", Imu, self.IMUCb)

        
        rospy.on_shutdown(self.saveDataOnShutdown)

        rospy.loginfo("RecordData Node is initialized...");


    def UAVPoseCb(self, msg):
        
        time = msg.header.stamp.to_sec()
        self.UAVPose_ = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.UAVVel_ = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self.UAVAtti_ = (msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        self.UAVLists_.append([time] + list(self.UAVPose_) + list(self.UAVVel_) + list(self.UAVAtti_))

    def IMUCb(self, msg):

        time = msg.header.stamp.to_sec()
        acc = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.IMULists_.append([time] + list(acc))

    def aprilTagCb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x + self.UAVPose_[0], msg.pose.pose.position.y + self.UAVPose_[1], msg.pose.pose.position.z + self.UAVPose_[2])
            velocity = (msg.twist.twist.linear.x + self.UAVVel_[0], msg.twist.twist.linear.y + self.UAVVel_[1], msg.twist.twist.linear.z + self.UAVVel_[2])
            self.aprilTagList_.append([time] + list(position) + list(velocity))
        
    def KF0Cb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
            UAV_uncertainty = (msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[2])
            UGV_uncertainty = (msg.pose.covariance[3], msg.pose.covariance[4], msg.pose.covariance[5])
            UGV_vel_uncertainty = (msg.twist.covariance[0], msg.twist.covariance[1], msg.twist.covariance[2])
            self.KF0List_.append([time] + list(position) + list(velocity))
            self.KF0UncertaintyList_.append([time] + list(UAV_uncertainty) + list(UGV_uncertainty) + list(UGV_vel_uncertainty))

    def KF1Cb(self, msg):
        
        if (len(self.UAVPose_) == 3):
            time = msg.header.stamp.to_sec()
            position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
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
        vel = (msg.velocity.x, msg.velocity.y, msg.velocity.z)
        self.VelSpList_.append([time] + list(vel))

    def thrSpCb(self, msg):

        time = rospy.Time.now().to_sec()
        thrust = msg.thrust
        atti = (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        self.thrSpList_.append([time] + [thrust] + list(atti))
    
    def saveDataOnShutdown(self):

        runWD = os.getcwd()
        cwd = runWD.split("/")[-1]
        assert( cwd == "autonomous_landing_ws" )
        target_path = f"../data/{self.aims}"
        os.chdir(target_path)

        if (self.aims == "kalman_tuning"):
            
            cols = ['time', 'x', 'y', 'z', 'vx', 'vy', 'vz']
            uncerCols = ['time' ,'pxx', 'pyy', 'pzz', 'paxx', 'payy', 'pazz', 'pa_vxx', 'pa_vyy', 'pa_vzz']
            GPSCols = ['time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'w', 'x', 'y', 'z']
            IMUcols = ['ax','ay','az'] 

            GTIdx = range(len(self.GTList_))
            dfGTData = pd.DataFrame(self.GTList_[:len(GTIdx)], columns=cols, index=GTIdx)

            aprilTagIdx = range(len(self.aprilTagList_))
            dfAprilTagData = pd.DataFrame(self.aprilTagList_[:len(aprilTagIdx)], columns=cols, index=aprilTagIdx)

            KF0Idx = range(len(self.KF0List_))
            dfKF0Data = pd.DataFrame(self.KF0List_[:len(KF0Idx)], columns=cols, index=KF0Idx)
            KF0Idx = range(len(self.KF0UncertaintyList_))
            dfKF0UncerData = pd.DataFrame(self.KF0UncertaintyList_[:len(KF0Idx)], columns=uncerCols, index=KF0Idx)
            
            KF1Idx = range(len(self.KF1List_))
            dfKF1Data = pd.DataFrame(self.KF1List_[:len(KF1Idx)], columns=cols, index=KF1Idx)
            KF1Idx = range(len(self.KF1UncertaintyList_))
            dfKF1UncerData = pd.DataFrame(self.KF1UncertaintyList_[:len(KF1Idx)], columns=uncerCols, index=KF1Idx)

            GPSIdx = range(len(self.UAVLists_))
            dfGPSData = pd.DataFrame(self.UAVLists_[:len(GPSIdx)], columns=GPSCols, index=GPSIdx)
            IMUIdx = range(len(self.IMULists_))
            dfIMUData = pd.DataFrame(self.IMULists_[:len(IMUIdx)], columns=IMUcols, index=IMUIdx)

            dfGTData.to_csv("GTData.csv", index=False)
            dfAprilTagData.to_csv("aprilTagData.csv", index=False)
            dfKF0Data.to_csv("KF0Data.csv", index=False)
            dfKF1Data.to_csv("KF1Data.csv", index=False)
            
            dfGPSData.to_csv("GPSData.csv", index=False)
            dfIMUData.to_csv("IMUData.csv", index=False)

            dfKF0UncerData.to_csv("KF0UncertaintyData.csv", index=False)
            dfKF1UncerData.to_csv("KF1UncertaintyData.csv", index=False)

        elif (self.aims == "velocity_tuning"):

            GPSCols = ['time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'w', 'x', 'y', 'z']
            vCols = ['time', 'vx', 'vy', 'vz']

            GPSIdx = range(len(self.UAVLists_))
            dfGPSData = pd.DataFrame(self.UAVLists_[:len(GPSIdx)], columns=GPSCols, index=GPSIdx)

            VelSpIdx = range(len(self.VelSpList_))
            dfVelSpData = pd.DataFrame(self.VelSpList_[:len(VelSpIdx)], columns=vCols, index=VelSpIdx)

            dfGPSData.to_csv("GPSData.csv", index=False)
            dfVelSpData.to_csv("VelSpData.csv", index=False)

        elif (self.aims == "sliding_throttle"):

            GPSCols = ['time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'w', 'x', 'y', 'z']
            thrCols = ['time', 'thr', 'w', 'x', 'y', 'z']
            IMUcols = ['time', 'ax','ay','az'] 
        
            GPSIdx = range(len(self.UAVLists_))
            dfGPSData = pd.DataFrame(self.UAVLists_[:len(GPSIdx)], columns=GPSCols, index=GPSIdx)
        
            ThrSpIdx = range(len(self.thrSpList_))
            dfThrSpData = pd.DataFrame(self.thrSpList_[:len(ThrSpIdx)], columns=thrCols, index=ThrSpIdx)

            IMUIdx = range(len(self.IMULists_))
            dfIMUData = pd.DataFrame(self.IMULists_[:len(IMUIdx)], columns=IMUcols, index=IMUIdx)

            dfGPSData.to_csv("GPSData.csv", index=False)
            dfThrSpData.to_csv("thrSpData.csv", index=False)
            dfIMUData.to_csv("IMUData.csv", index=False)


        rospy.loginfo("Data saved to CSV files on shutdown.")


def main():
    rospy.init_node("record_node", anonymous=True)
    aims = ["kalman_tuning", "sliding_throttle", "velocity_tuning"]
    idx = 1
    recordDataObj = RecordData(aims[idx])
    rospy.loginfo(f"{aims[idx]} is recording")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
