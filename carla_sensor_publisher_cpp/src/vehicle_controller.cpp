// Copyright (c) 2025. MIT License.
// vehicle_controller.cpp — GaeControlCmd subscriber + PID speed control

#include "carla_sensor_publisher/vehicle_controller.hpp"

#include <carla/client/Vehicle.h>

#include <cmath>
#include <algorithm>
#include <iostream>

namespace carla_sensor_publisher {

VehicleController::VehicleController(carla::SharedPtr<carla::client::Actor> vehicle,
                                     rclcpp::Node::SharedPtr node)
    : vehicle_(vehicle), node_(node),
      speed_pid_(0.5, 0.05, 0.1, 0.0, 1.0),
      brake_pid_(0.3, 0.0, 0.05, 0.0, 1.0) {

#ifdef GAE_MSGS_AVAILABLE
    if (node) {
        sub_ = node->create_subscription<gae_msgs::msg::GaeControlCmd>(
            TOPIC, 10,
            [this](const gae_msgs::msg::GaeControlCmd::SharedPtr msg) {
                this->onCmd(msg);
            });

        timer_ = node->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            [this]() { this->controlLoop(); });

        enabled_ = true;
        std::cout << "  ✓ VehicleController: " << TOPIC
                  << " dinleniyor (PID hız kontrolü)" << std::endl;
    } else
#endif
    {
        std::cout << "  ✗ VehicleController: gae_msgs yok veya ROS kapalı, "
                  << "araç kontrolü devre dışı" << std::endl;
    }
}

VehicleController::~VehicleController() {
    destroy();
}

void VehicleController::destroy() {
    enabled_ = false;
    try {
        auto vehicle = std::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_);
        if (vehicle) {
            carla::rpc::VehicleControl ctrl;
            ctrl.throttle = 0.0f;
            ctrl.brake = 1.0f;
            ctrl.steer = 0.0f;
            vehicle->ApplyControl(ctrl);
        }
    } catch (...) {}
}

#ifdef GAE_MSGS_AVAILABLE
void VehicleController::onCmd(const gae_msgs::msg::GaeControlCmd::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(lock_);
    last_cmd_ = msg;
    last_cmd_time_ = std::chrono::steady_clock::now();
    has_cmd_ = true;
}
#endif

double VehicleController::getCurrentSpeedKmh() {
    auto vehicle = std::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_);
    if (!vehicle) return 0.0;
    auto v = vehicle->GetVelocity();
    double speed_ms = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return speed_ms * 3.6;
}

void VehicleController::controlLoop() {
    if (!enabled_) return;

#ifdef GAE_MSGS_AVAILABLE
    gae_msgs::msg::GaeControlCmd::SharedPtr cmd;
    bool has_cmd;
    std::chrono::steady_clock::time_point cmd_time;

    {
        std::lock_guard<std::mutex> lk(lock_);
        cmd = last_cmd_;
        has_cmd = has_cmd_;
        cmd_time = last_cmd_time_;
    }

    if (!has_cmd || !cmd) return;

    // Timeout check
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - cmd_time).count();
    if (elapsed > cmd_timeout_) {
        auto vehicle = std::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_);
        if (vehicle) {
            carla::rpc::VehicleControl ctrl;
            ctrl.throttle = 0.0f;
            ctrl.brake = 1.0f;
            ctrl.steer = 0.0f;
            ctrl.hand_brake = false;
            ctrl.reverse = false;
            vehicle->ApplyControl(ctrl);
        }
        return;
    }

    // ── Steering conversion ──────────────────────────────────────────
    // GaeControlCmd: 0=full right, 1800=straight, 3600=full left
    // CARLA steer:  +1.0=full right, 0=straight, -1.0=full left
    float steer = static_cast<float>((1800.0 - cmd->steering) / 1800.0);
    steer = std::clamp(steer, -1.0f, 1.0f);

    // ── Gear / Reverse ───────────────────────────────────────────────
    bool reverse = (cmd->gear == 2);

    // ── Target speed and PID ─────────────────────────────────────────
    double target_speed_kmh = static_cast<double>(cmd->throttle) * RPM_TO_KMH;
    double current_speed_kmh = getCurrentSpeedKmh();
    double speed_error = target_speed_kmh - current_speed_kmh;

    double dt = 0.05;  // 20 Hz

    // Manual brake (0-10000)
    double manual_brake = static_cast<double>(cmd->brake) / 10000.0;

    float carla_throttle, carla_brake;

    if (cmd->throttle == 0 && manual_brake < 0.01) {
        carla_throttle = 0.0f;
        carla_brake = 0.05f;
    } else if (manual_brake > 0.01) {
        carla_throttle = 0.0f;
        carla_brake = static_cast<float>(std::max(manual_brake, 0.1));
        speed_pid_.reset();
    } else if (speed_error > 0) {
        carla_throttle = static_cast<float>(speed_pid_.step(speed_error, dt));
        carla_brake = 0.0f;
    } else {
        carla_throttle = 0.0f;
        carla_brake = static_cast<float>(brake_pid_.step(-speed_error, dt));
        speed_pid_.reset();
    }

    bool hand_brake = static_cast<bool>(cmd->mechanical_brake);

    // ── Apply CARLA control ──────────────────────────────────────────
    auto vehicle = std::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_);
    if (vehicle) {
        carla::rpc::VehicleControl ctrl;
        ctrl.throttle = carla_throttle;
        ctrl.steer = steer;
        ctrl.brake = carla_brake;
        ctrl.hand_brake = hand_brake;
        ctrl.reverse = reverse;
        vehicle->ApplyControl(ctrl);
    }
#endif
}

}  // namespace carla_sensor_publisher
