// Copyright (c) 2025. MIT License.
// gnss_publisher.hpp — Dual GNSS sensor publisher
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <carla/client/Sensor.h>
#include <carla/client/World.h>

namespace carla_sensor_publisher {

class GNSSPublisher {
public:
    GNSSPublisher(carla::SharedPtr<carla::client::Actor> parent,
                  rclcpp::Node::SharedPtr node = nullptr);
    ~GNSSPublisher();
    void destroy();

    double getLat() const { return lat_; }
    double getLon() const { return lon_; }

private:
    void onFront(carla::SharedPtr<carla::sensor::SensorData> data);
    void onRear(carla::SharedPtr<carla::sensor::SensorData> data);
    void publishGNSS(carla::SharedPtr<carla::sensor::SensorData> data,
                     const std::string& frame_id, const std::string& side);
    builtin_interfaces::msg::Time wallClockStamp();

    carla::SharedPtr<carla::client::Actor> sensor_front_;
    carla::SharedPtr<carla::client::Actor> sensor_rear_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_front_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_rear_;

    double lat_ = 0.0;
    double lon_ = 0.0;
};

}  // namespace carla_sensor_publisher
