cmake_minimum_required(VERSION 3.8)
project(miapr_ur5e)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(include)
# Add dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(miapr_ur5e_interfaces REQUIRED)

include_directories(
  include
  /opt/ros/humble/include
  /opt/ros/humble/include/urdf
  /opt/ros/humble/include/moveit_msgs
  /opt/ros/humble/include/geometry_msgs
  /opt/ros/humble/include/trajectory_msgs
  /opt/ros/humble/include/sensor_msgs
  /opt/ros/humble/include/visualization_msgs
  /opt/ros/humble/include/object_recognition_msgs
  /opt/ros/humble/include/shape_msgs
  /opt/ros/humble/include/octomap_msgs
  /opt/ros/humble/include/unique_identifier_msgs
  /opt/ros/humble/include/action_msgs
  /opt/ros/humble/include/rclcpp_action
  /opt/ros/humble/include/rcl_action
  /opt/ros/humble/include/tf2_msgs
  /opt/ros/humble/include/pluginlib
  /opt/ros/humble/include/class_loader
  /opt/ros/humble/include/graph_msgs
  ${colcon_INCLUDE_DIRS}
)

# Add executables
add_executable(joint_space_trajectory src/joint_space_trajectory.cpp)
ament_target_dependencies(joint_space_trajectory rclcpp moveit_ros_planning_interface)

add_executable(trajectory_control_server src/trajectory_control_server.cpp)
ament_target_dependencies(trajectory_control_server rclcpp moveit_ros_planning_interface miapr_ur5e_interfaces)

# Install targets
install(TARGETS
  joint_space_trajectory
  trajectory_control_server
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
