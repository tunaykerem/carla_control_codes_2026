cmake_minimum_required(VERSION 3.14)
project(ros2_publisher_gcc)

# Standard ROS 2 compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# gae_msgs — optional
find_package(gae_msgs QUIET)
if(gae_msgs_FOUND)
  message(STATUS "gae_msgs FOUND")
  add_definitions(-DGAE_MSGS_AVAILABLE)
endif()

include_directories(
  include
  ../common
)

set(SOURCES
  src/main.cpp
  src/tcp_receiver.cpp
  src/ouster_lidar_publisher.cpp
  src/velodyne_lidar_publisher.cpp
  src/zed_camera_publisher.cpp
  src/imu_publisher.cpp
  src/gnss_publisher.cpp
  src/transforms.cpp
)

add_executable(ros2_publisher_node ${SOURCES})

set(ROS_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
  tf2
  tf2_ros
  cv_bridge
)
if(gae_msgs_FOUND)
  list(APPEND ROS_DEPS gae_msgs)
endif()

ament_target_dependencies(ros2_publisher_node ${ROS_DEPS})
target_link_libraries(ros2_publisher_node ${OpenCV_LIBS} pthread)

install(TARGETS ros2_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
