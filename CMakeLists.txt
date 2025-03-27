cmake_minimum_required(VERSION 3.5)
project(racecar_interface)

# If your package contains C++ nodes, set the standard (adjust if necessary)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(racecar_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------------------------------------------------------
# Generate interfaces for services (and messages if any)
# ---------------------------------------------------------------------------
# In ROS2, instead of catkin's generate_messages/message_generation,
# we use rosidl_generate_interfaces to create service (and message) interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Reset.srv"
  "srv/ResetObstacle.srv"
  DEPENDENCIES std_msgs
)

# ---------------------------------------------------------------------------
# Installation of Python executables
# ---------------------------------------------------------------------------
# In ROS2, Python nodes are installed into lib/<package_name>
install(PROGRAMS
  script/visualization_node.py
  script/simulation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize the package with ament
ament_package()
