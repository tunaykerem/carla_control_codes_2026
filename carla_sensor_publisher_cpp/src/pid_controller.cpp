// Copyright (c) 2025. MIT License.
// pid_controller.cpp — Simple PID controller

#include "carla_sensor_publisher/pid_controller.hpp"
#include <algorithm>

namespace carla_sensor_publisher {

PIDController::PIDController(double kp, double ki, double kd,
                             double out_min, double out_max)
    : kp_(kp), ki_(ki), kd_(kd),
      out_min_(out_min), out_max_(out_max),
      integral_(0.0), prev_error_(0.0) {}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}

double PIDController::step(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    integral_ += error * dt;
    // Anti-windup
    integral_ = std::clamp(integral_, -5.0, 5.0);

    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return std::clamp(output, out_min_, out_max_);
}

}  // namespace carla_sensor_publisher
