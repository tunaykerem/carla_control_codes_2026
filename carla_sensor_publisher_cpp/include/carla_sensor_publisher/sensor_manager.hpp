// Copyright (c) 2025. MIT License.
// sensor_manager.hpp — Manages all sensors on the vehicle
#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <carla/client/Actor.h>
#include <carla/client/World.h>

#include "carla_sensor_publisher/ouster_lidar.hpp"
#include "carla_sensor_publisher/velodyne_lidar.hpp"
#include "carla_sensor_publisher/zed_camera.hpp"
#include "carla_sensor_publisher/imu_publisher.hpp"
#include "carla_sensor_publisher/gnss_publisher.hpp"

namespace carla_sensor_publisher {

class SensorManager {
public:
    SensorManager(carla::client::World& world,
                  carla::SharedPtr<carla::client::Actor> vehicle,
                  rclcpp::Node::SharedPtr node = nullptr);
    ~SensorManager();
    void destroy();

    /// Broadcast static TFs from base_link to all sensor frames
    void broadcastStaticTFs(rclcpp::Node::SharedPtr node);

private:
    carla::SharedPtr<carla::client::Actor> vehicle_;

    std::unique_ptr<OusterLidar>    ouster_;
    std::unique_ptr<VelodyneLidar>  velodyne_;
    std::unique_ptr<ZedCamera>      zed_;
    std::unique_ptr<IMUPublisher>   imu_;
    std::unique_ptr<GNSSPublisher>  gnss_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

}  // namespace carla_sensor_publisher
