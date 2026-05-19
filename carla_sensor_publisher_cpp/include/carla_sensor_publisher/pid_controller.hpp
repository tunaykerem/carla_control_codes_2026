// Copyright (c) 2025. MIT License.
// pid_controller.hpp — Simple PID controller
#pragma once

namespace carla_sensor_publisher {

class PIDController {
public:
    PIDController(double kp = 0.5, double ki = 0.05, double kd = 0.1,
                  double out_min = 0.0, double out_max = 1.0);

    void reset();
    double step(double error, double dt);

private:
    double kp_, ki_, kd_;
    double out_min_, out_max_;
    double integral_;
    double prev_error_;
};

}  // namespace carla_sensor_publisher
