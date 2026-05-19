// Copyright (c) 2025. MIT License.
// transforms.cpp — URDF TF definitions and CARLA transform conversion

#include "carla_sensor_publisher/transforms.hpp"

namespace carla_sensor_publisher {

const std::map<std::string, SensorPose>& getTransforms() {
    // [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad] — CARLA coordinate system
    // ROS/URDF Y=left, CARLA Y=right → all Y values are inverted from URDF
    static const std::map<std::string, SensorPose> transforms = {
        // LiDAR — 360° horizontal FOV, yaw irrelevant
        {"velodyne",         {0.80,  0.00, 1.25,  0.0,  0.0, 0.0}},

        // Ouster: URDF yaw=π but 360° LiDAR → yaw unchanged
        {"ouster",           {0.85,  0.00, 1.10,  0.0,  0.0, 0.0}},

        // ZED 2 — forward-facing (yaw=0)
        {"zed_left",         {0.85, -0.06, 1.00,  0.0,  0.0, 0.0}},
        {"zed_right",        {0.85,  0.06, 1.00,  0.0,  0.0, 0.0}},

        // IMU — URDF y=-0.12 → CARLA y=+0.12
        {"imu_1",            {0.40,  0.12, 0.80,  0.0,  0.0, 0.0}},
        {"imu_2",            {0.40,  0.00, 0.80,  0.0,  0.0, 0.0}},

        // GNSS — URDF y=-0.45 (right in ROS) → CARLA y=+0.45 (right in CARLA)
        {"gnss_front_right", {1.35,  0.45, 0.30,  0.0,  0.0, 0.0}},
        {"gnss_rear_right",  {0.00,  0.45, 0.30,  0.0,  0.0, 0.0}},
    };
    return transforms;
}

carla::geom::Transform urdfToCarlaTransform(const SensorPose& pose,
                                             double yaw_extra_deg) {
    double x     = pose[0];
    double y     = pose[1];
    double z     = pose[2];
    double roll  = pose[3];
    double pitch = pose[4];
    double yaw   = pose[5];

    constexpr double RAD2DEG = 180.0 / M_PI;

    return carla::geom::Transform(
        carla::geom::Location(
            static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(z)),
        carla::geom::Rotation(
            static_cast<float>(pitch * RAD2DEG),          // pitch
            static_cast<float>(yaw * RAD2DEG + yaw_extra_deg),  // yaw
            static_cast<float>(roll * RAD2DEG))           // roll
    );
}

}  // namespace carla_sensor_publisher
