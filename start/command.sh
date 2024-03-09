#! /bin/bash

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

# Give some time for the drone to go over the tag
echo "Sleeping 5 seconds"
sleep 5

# Move the UGV
echo "Moving the UGV ..."
rostopic pub -r 10 cmd_vel geometry_msgs/Twist "linear:
  x: 3.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# monitoring the image
echo "Opening the image ..."
rosrun image_view image_view image:=/tag_detections_image

# Publish the velocity setpoint
echo "Publish the velocity setpoint ..."
rostopic pub -r 20 mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear: {x: 12.0}"

# Publish Yaw and Publish velocity
# MASK_POSITION = 0b0000011111111000, // x, y, z, vyaw
# MASK_VELOCITY = 0b0000011111000111, // vx, vy, vz, vyaw
rostopic pub -r 20 /mavros/setpoint_raw/local mavros_msgs/PositionTarget "{coordinate_frame: 8, type_mask: 0b0000011111000111, velocity: {x: -0.5}, yaw: 1.0}"
