cmake_minimum_required(VERSION 3.14)
project(carla_sensor_publisher)

# 1. C++20 ve LLVM libc++ Standartlarını En Üstte Genel Olarak Tanımlıyoruz
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-std=c++20 -stdlib=libc++)
add_link_options(-stdlib=libc++ -lc++abi)

# ── ROS 2 deps ──────────────────────────────────────────────────────────────
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

# gae_msgs — optional: if not found, vehicle controller is disabled
find_package(gae_msgs QUIET)
if(gae_msgs_FOUND)
  message(STATUS "gae_msgs FOUND — VehicleController enabled")
  add_definitions(-DGAE_MSGS_AVAILABLE)
else()
  message(WARNING "gae_msgs NOT found — VehicleController disabled")
endif()

# ── CARLA C++ SDK ───────────────────────────────────────────────────────────
set(CARLA_ROOT "$ENV{CARLA_ROOT}" CACHE PATH "Path to CARLA source tree (CarlaUE5)")
if(NOT CARLA_ROOT OR "${CARLA_ROOT}" STREQUAL "")
  set(CARLA_ROOT "/home/tunay/itu_sct_gae/karla_zimulazyon/CarlaUE5")
endif()

set(CARLA_LIBCARLA_INCLUDE "${CARLA_ROOT}/LibCarla/source")
set(CARLA_LIBCARLA_LIB     "${CARLA_ROOT}/Build/LibCarla")
set(CARLA_DEPS             "${CARLA_ROOT}/Build/_deps")

# Boost (modular)
file(GLOB BOOST_MODULE_DIRS "${CARLA_DEPS}/boost-src/libs/*/include")

# rpclib
set(CARLA_RPC_INCLUDE   "${CARLA_DEPS}/rpclib-src/include")
set(CARLA_RPC_LIB       "${CARLA_DEPS}/rpclib-build/librpc.a")

# Recast Navigation
set(CARLA_RECAST_INCLUDE_RECAST  "${CARLA_DEPS}/recastnavigation-src/Recast/Include")
set(CARLA_RECAST_INCLUDE_DETOUR  "${CARLA_DEPS}/recastnavigation-src/Detour/Include")
set(CARLA_RECAST_LIB_RECAST     "${CARLA_DEPS}/recastnavigation-build/Recast/libRecast.a")
set(CARLA_RECAST_LIB_DETOUR     "${CARLA_DEPS}/recastnavigation-build/Detour/libDetour.a")
set(CARLA_RECAST_LIB_DETOURCROWD "${CARLA_DEPS}/recastnavigation-build/DetourCrowd/libDetourCrowd.a")

# ── Include dirs ────────────────────────────────────────────────────────────
include_directories(
  include
  ${CARLA_LIBCARLA_INCLUDE}
  ${BOOST_MODULE_DIRS}
  ${CARLA_RPC_INCLUDE}
  ${CARLA_RECAST_INCLUDE_RECAST}
  ${CARLA_RECAST_INCLUDE_DETOUR}
)

# ── Source files ────────────────────────────────────────────────────────────
set(SOURCES
  src/main.cpp
  src/transforms.cpp
  src/pid_controller.cpp
  src/ouster_lidar.cpp
  src/velodyne_lidar.cpp
  src/zed_camera.cpp
  src/imu_publisher.cpp
  src/gnss_publisher.cpp
  src/vehicle_controller.cpp
  src/sensor_manager.cpp
)

# Executable burada oluşturuluyor
add_executable(carla_sensor_node ${SOURCES})

# DÜZELTME: target_... ayarlarını add_executable'dan SONRA tanımlıyoruz
target_compile_options(carla_sensor_node PRIVATE -stdlib=libc++)
target_link_options(carla_sensor_node PRIVATE -stdlib=libc++ -lc++abi)

# ── Link CARLA static libraries ───────────────────────────────────────────
target_link_libraries(carla_sensor_node
  ${CARLA_LIBCARLA_LIB}/libcarla-client.a
  ${CARLA_RPC_LIB}
  ${CARLA_RECAST_LIB_DETOUR}
  ${CARLA_RECAST_LIB_DETOURCROWD}
  ${CARLA_RECAST_LIB_RECAST}
  pthread
)

# Link Boost static libs
set(BOOST_BUILD "${CARLA_DEPS}/boost-build/libs")
target_link_libraries(carla_sensor_node
  ${BOOST_BUILD}/filesystem/libboost_filesystem.a
)

# ── ROS 2 ament deps ───────────────────────────────────────────────────────
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

ament_target_dependencies(carla_sensor_node ${ROS_DEPS})
target_link_libraries(carla_sensor_node ${OpenCV_LIBS})

# ── Install ─────────────────────────────────────────────────────────────────
install(TARGETS carla_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()