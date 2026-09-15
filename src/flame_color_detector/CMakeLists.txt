cmake_minimum_required(VERSION 2.8.3)
project(flame_color_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/flame_color_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
