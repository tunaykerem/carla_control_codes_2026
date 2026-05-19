// Copyright (c) 2025. MIT License.
// ouster_lidar.hpp — Ouster OS0-64 LiDAR with sector accumulation
#pragma once

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <carla/client/Sensor.h>
#include <carla/client/World.h>

namespace carla_sensor_publisher {

class OusterLidar {
public:
    static constexpr const char* TOPIC    = "/ouster/points";
    static constexpr const char* FRAME_ID = "ouster";
    static constexpr int NUM_CHANNELS     = 64;
    static constexpr int NUM_SECTORS      = 12;
    static constexpr double STALE_S       = 0.20;
    static constexpr int POINT_STEP       = 48;

    OusterLidar(carla::SharedPtr<carla::client::Actor> parent,
                rclcpp::Node::SharedPtr node = nullptr);
    ~OusterLidar();
    void destroy();

private:
    void onSensorData(carla::SharedPtr<carla::sensor::SensorData> data);
    void processAndPublish(const float* raw_pts, size_t n_total, double timestamp);
    std::vector<sensor_msgs::msg::PointField> buildFields();
    builtin_interfaces::msg::Time wallClockStamp();

    carla::SharedPtr<carla::client::Actor> sensor_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    std::mutex busy_;
    // Sector accumulation
    std::unordered_map<int, std::vector<float>> sector_grid_;   // sector_idx → [x,y,z,i, ...]
    std::unordered_map<int, double>             sector_stamp_;  // sector_idx → CARLA timestamp

    std::vector<sensor_msgs::msg::PointField> fields_;

    // Profiling
    static constexpr int PROF_EVERY = 50;
    std::atomic<int> prof_count_{0};
    std::atomic<int> prof_dropped_{0};
};

}  // namespace carla_sensor_publisher
