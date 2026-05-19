// Copyright (c) 2025. MIT License.
// transforms.hpp — URDF TF definitions for CARLA sensor placement
#pragma once

#include <array>
#include <map>
#include <string>
#include <cmath>

#include <carla/geom/Transform.h>

namespace carla_sensor_publisher {

/// URDF TF: [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad] — CARLA coordinate system
using SensorPose = std::array<double, 6>;

/// All sensor poses (CARLA coords: X=forward, Y=right, Z=up)
const std::map<std::string, SensorPose>& getTransforms();

/// Convert URDF xyzrpy to carla::geom::Transform
carla::geom::Transform urdfToCarlaTransform(const SensorPose& pose,
                                             double yaw_extra_deg = 0.0);

}  // namespace carla_sensor_publisher
