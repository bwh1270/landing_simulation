#! /bin/bash

# After starting /home/hyunee/AIMS/PX4_PPNNPN/PX4-Autopilot/run_px4RosGazebo.sh

# Gimbal Control
echo "Camera Down ..."
rostopic pub --once /mavros/mount_control/command mavros_msgs/MountControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
mode: 2
pitch: -90.0
roll: 0.0
yaw: 0.0
altitude: 0.0
latitude: 0.0
longitude: 0.0"

# Set OFFBOARD mode
echo "Mode is changing to Offboard ..."
rostopic pub --once /aims/start std_msgs/Bool "data: true"

# Give some time for the drone to go over the tag
echo "Sleeping 5 seconds"
sleep 5

Move the UGV
echo "Moving the UGV ..."
rostopic pub -r 10 cmd_vel geometry_msgs/Twist "linear:
  x: 3.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"