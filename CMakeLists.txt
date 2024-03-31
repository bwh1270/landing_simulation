cmake_minimum_required(VERSION 3.0.2)
project(landing_simulation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

add_message_files(
    FILES
    wind.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES landing_simulation
   CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs tf2 tf2_ros tf2_geometry_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_executable(apriltag_node
    src/AprilTagDetection.cpp
    src/Mathematics.cpp
)
add_executable(kf_node
    src/LinearKalmanFilter.cpp
    src/Mathematics.cpp
)
add_executable(pc_node
    src/PositionControl.cpp
    src/Mathematics.cpp
)
add_executable(ugv_vc_node
    src/UGVVelocityControl.cpp
)
add_executable(setpoint_publisher_node
    src/SetpointPublisher.cpp
)
add_executable(wind_generator_node
    src/WindGenerator.cpp
)
# add_executable(tf_publisher_node
#     src/test/tf_talker_class_node.cpp
# )
# add_executable(control_test_node 
#     src/test/control_test.cpp
# )
# add_executable(control_node
#     src/test/high_level_control_class.cpp
# )


target_link_libraries(apriltag_node ${catkin_LIBRARIES})
target_link_libraries(kf_node ${catkin_LIBRARIES})
target_link_libraries(pc_node ${catkin_LIBRARIES})
target_link_libraries(ugv_vc_node ${catkin_LIBRARIES})
target_link_libraries(setpoint_publisher_node ${catkin_LIBRARIES})
target_link_libraries(wind_generator_node ${catkin_LIBRARIES})
# target_link_libraries(tf_publisher_node ${catkin_LIBRARIES})
# target_link_libraries(control_test_node ${catkin_LIBRARIES})
# target_link_libraries(control_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
   scripts/recordData.py
   scripts/windGenerationTest.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
