cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoiding_robot)

## Package Dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  vnh5019_motor_controller
  dynamic_reconfigure
)

## Uncomment this if the package has a setup.py. This macro ensures
catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
	cfg/Parameters.cfg
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES obstacle_avoiding_robot
  #  CATKIN_DEPENDS rospy std_msgs vnh5019_motor_controller
  #  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  nodes/robot_control 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/robot_control.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

