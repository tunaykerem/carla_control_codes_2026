// Copyright (c) 2025. MIT License.
// vehicle_controller.hpp — GaeControlCmd subscriber + PID speed control
#pragma once

#include <memory>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#ifdef GAE_MSGS_AVAILABLE
#include <gae_msgs/msg/gae_control_cmd.hpp>
#endif

#include <carla/client/Vehicle.h>
#include <carla/rpc/VehicleControl.h>

#include "carla_sensor_publisher/pid_controller.hpp"

namespace carla_sensor_publisher {

class VehicleController {
public:
    static constexpr const char* TOPIC = "/vehicle/control";
    static constexpr double RPM_TO_KMH = 35.0 / 200.0;  // 0.175

    VehicleController(carla::SharedPtr<carla::client::Actor> vehicle,
                      rclcpp::Node::SharedPtr node);
    ~VehicleController();
    void destroy();

private:
#ifdef GAE_MSGS_AVAILABLE
    void onCmd(const gae_msgs::msg::GaeControlCmd::SharedPtr msg);
#endif
    void controlLoop();
    double getCurrentSpeedKmh();

    carla::SharedPtr<carla::client::Actor> vehicle_;
    rclcpp::Node::SharedPtr node_;
    bool enabled_ = false;

    PIDController speed_pid_;
    PIDController brake_pid_;

    std::mutex lock_;
#ifdef GAE_MSGS_AVAILABLE
    gae_msgs::msg::GaeControlCmd::SharedPtr last_cmd_;
    rclcpp::Subscription<gae_msgs::msg::GaeControlCmd>::SharedPtr sub_;
#endif
    std::chrono::steady_clock::time_point last_cmd_time_;
    bool has_cmd_ = false;
    double cmd_timeout_ = 1.0;

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace carla_sensor_publisher
