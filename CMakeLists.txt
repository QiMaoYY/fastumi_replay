cmake_minimum_required(VERSION 3.0.2)
project(fastumi_replay)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/clamp_visualizer.py
  scripts/replay_session.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  meshes
  rviz
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
