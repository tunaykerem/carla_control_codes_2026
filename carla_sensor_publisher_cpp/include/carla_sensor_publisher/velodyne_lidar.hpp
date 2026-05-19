// Copyright (c) 2025. MIT License.
// velodyne_lidar.hpp — Velodyne LiDAR fast mode
#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <carla/client/Sensor.h>
#include <carla/client/World.h>

namespace carla_sensor_publisher {

class VelodyneLidar {
public:
    static constexpr const char* TOPIC    = "/velodyne/points";
    static constexpr const char* FRAME_ID = "velodyne";
    static constexpr int NUM_CHANNELS     = 16;
    static constexpr int POINT_STEP       = 20;

    VelodyneLidar(carla::SharedPtr<carla::client::Actor> parent,
                  rclcpp::Node::SharedPtr node = nullptr);
    ~VelodyneLidar();
    void destroy();

private:
    void onSensorData(carla::SharedPtr<carla::sensor::SensorData> data);
    std::vector<sensor_msgs::msg::PointField> buildFields();
    builtin_interfaces::msg::Time wallClockStamp();

    carla::SharedPtr<carla::client::Actor> sensor_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::vector<sensor_msgs::msg::PointField> fields_;
};

}  // namespace carla_sensor_publisher
