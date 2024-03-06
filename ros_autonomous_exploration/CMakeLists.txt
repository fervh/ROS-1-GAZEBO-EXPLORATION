cmake_minimum_required(VERSION 3.0.2)
project(ros_autonomous_exploration)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS GAZEBO
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/porcentaje_borde_node.py
  src/exploration_node.py
  src/target_point_node.py
  src/map_save_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)