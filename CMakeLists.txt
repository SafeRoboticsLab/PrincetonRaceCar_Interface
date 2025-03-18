cmake_minimum_required(VERSION 3.5)  # Updated minimum CMake version for ROS 2
project(racecar_interface)

# Find necessary packages in ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Replaces rospy
find_package(std_msgs REQUIRED)
find_package(racecar_msgs REQUIRED)
find_package(tf2_ros REQUIRED)  # Replaces tf2 in ROS 1
find_package(tf2_geometry_msgs REQUIRED)  # Needed for tf2 message transformations
find_package(dynamic_reconfigure REQUIRED)

# Uncomment this if the package has a setup.py. This macro ensures
# modules and global scripts declared therein get installed.
# In ROS 2, `ament_python_install_package()` is used instead of catkin.
# See: https://docs.ros.org/en/foxy/Tutorials/Ament-Python-Package.html
# ament_python_install_package(${PROJECT_NAME})

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services, or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g., std_msgs, actionlib_msgs, ...).
## * In package.xml:
##   * add a build_depend tag for "rosidl_default_generators"
##   * add a build_depend and an exec_depend tag for each package in MSG_DEP_SET
## * In this file (CMakeLists.txt):
##   * add "rosidl_default_generators" and every package in MSG_DEP_SET to
##     find_package(ament_cmake REQUIRED ...)
##   * uncomment the rosidl_generate_interfaces section below
##     and list every .msg/.srv/.action file to be processed

## Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Reset.srv"
  "srv/ResetObstacle.srv"
  DEPENDENCIES std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and an exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to find_package(ament_cmake REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/sim.cfg
)

###################################
## ROS 2 specific configuration ##
###################################
## ament_package() generates CMake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: Uncomment this if your package contains header files
## LIBRARIES: Libraries you create in this project that dependent projects also need
## DEPENDENCIES: System dependencies of this project that dependent projects also need

ament_export_dependencies(rclpy std_msgs racecar_msgs tf2_ros tf2_geometry_msgs dynamic_reconfigure)
ament_package()

######################
## Install Scripts ##
######################

# Install Python scripts (ROS 2 replaces `catkin_install_python` with `install()`)
install(PROGRAMS
  script/visualization_node.py
  script/simulation_node.py
  DESTINATION lib/${PROJECT_NAME}
)
