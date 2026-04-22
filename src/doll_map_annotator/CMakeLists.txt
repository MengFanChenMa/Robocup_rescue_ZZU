cmake_minimum_required(VERSION 2.8.3)
project(doll_map_annotator)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_message_files(
  FILES
  DollDetection.msg
  DollLandmark.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    message_runtime
    rospy
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/doll_detector.py
  scripts/doll_mapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
