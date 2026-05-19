cmake_minimum_required(VERSION 3.14)
project(carla_reader_clang)

# C++20 and LLVM libc++ for CARLA compatibility
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Force Clang compiler (can be overridden by environment variables)
# set(CMAKE_CXX_COMPILER "clang++")

add_compile_options(-std=c++20 -stdlib=libc++)
add_link_options(-stdlib=libc++ -lc++abi)

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
  ../common
  ${CARLA_LIBCARLA_INCLUDE}
  ${BOOST_MODULE_DIRS}
  ${CARLA_RPC_INCLUDE}
  ${CARLA_RECAST_INCLUDE_RECAST}
  ${CARLA_RECAST_INCLUDE_DETOUR}
)

# ── Source files ────────────────────────────────────────────────────────────
set(SOURCES
  src/main.cpp
  src/tcp_sender.cpp
  src/sensor_manager.cpp
  src/ouster_lidar_reader.cpp
  src/velodyne_lidar_reader.cpp
  src/zed_camera_reader.cpp
  src/gnss_reader.cpp
  src/imu_reader.cpp
  src/vehicle_controller.cpp
)

add_executable(carla_reader_node ${SOURCES})

target_compile_options(carla_reader_node PRIVATE -stdlib=libc++)
target_link_options(carla_reader_node PRIVATE -stdlib=libc++ -lc++abi)

# ── Link CARLA static libraries ───────────────────────────────────────────
target_link_libraries(carla_reader_node
  ${CARLA_LIBCARLA_LIB}/libcarla-client.a
  ${CARLA_RPC_LIB}
  ${CARLA_RECAST_LIB_DETOUR}
  ${CARLA_RECAST_LIB_DETOURCROWD}
  ${CARLA_RECAST_LIB_RECAST}
  pthread
)

# Link Boost static libs
set(BOOST_BUILD "${CARLA_DEPS}/boost-build/libs")
target_link_libraries(carla_reader_node
  ${BOOST_BUILD}/filesystem/libboost_filesystem.a
)
