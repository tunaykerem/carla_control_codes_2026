// Copyright (c) 2025. MIT License.
// imu_publisher.cpp — Dual IMU sensor publisher with CARLA→ROS coordinate conversion

#include "carla_sensor_publisher/imu_publisher.hpp"
#include "carla_sensor_publisher/transforms.hpp"

#include <carla/sensor/data/IMUMeasurement.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorBlueprint.h>

#include <cmath>
#include <chrono>
#include <algorithm>
#include <iostream>

namespace carla_sensor_publisher {

static builtin_interfaces::msg::Time makeWallClockStamp() {
    auto now = std::chrono::system_clock::now();
    auto epoch = now.time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch) -
                std::chrono::duration_cast<std::chrono::nanoseconds>(sec);
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(sec.count());
    t.nanosec = static_cast<uint32_t>(nsec.count());
    return t;
}

IMUPublisher::IMUPublisher(carla::SharedPtr<carla::client::Actor> parent,
                           rclcpp::Node::SharedPtr node)
    : node_(node) {

    auto world = parent->GetWorld();
    auto bp_lib = world.GetBlueprintLibrary();
    auto bp = bp_lib->Find("sensor.other.imu");
    carla::client::ActorBlueprint mutable_bp = *bp;
    mutable_bp.SetAttribute("sensor_tick", "0.005");  // 200 Hz

    auto tf1 = urdfToCarlaTransform(getTransforms().at("imu_1"));
    auto tf2 = urdfToCarlaTransform(getTransforms().at("imu_2"));

    sensor1_ = world.SpawnActor(mutable_bp, tf1, parent.get(),
                                carla::rpc::AttachmentType::Rigid);
    sensor2_ = world.SpawnActor(mutable_bp, tf2, parent.get(),
                                carla::rpc::AttachmentType::Rigid);

    if (node) {
        auto qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .history(rclcpp::HistoryPolicy::KeepLast);
        pub1_ = node->create_publisher<sensor_msgs::msg::Imu>("/sbg/ros/imu/data", qos);
        pub2_ = node->create_publisher<sensor_msgs::msg::Imu>("/imu/imu2/data", qos);
    }

    auto s1 = std::dynamic_pointer_cast<carla::client::Sensor>(sensor1_);
    if (s1) {
        s1->Listen([this](auto data) { this->onIMU1(data); });
    }
    auto s2 = std::dynamic_pointer_cast<carla::client::Sensor>(sensor2_);
    if (s2) {
        s2->Listen([this](auto data) { this->onIMU2(data); });
    }
}

IMUPublisher::~IMUPublisher() {
    destroy();
}

void IMUPublisher::destroy() {
    for (auto& sensor : {std::ref(sensor1_), std::ref(sensor2_)}) {
        if (sensor.get()) {
            try {
                auto s = std::dynamic_pointer_cast<carla::client::Sensor>(sensor.get());
                if (s && s->IsListening()) s->Stop();
                sensor.get()->Destroy();
            } catch (...) {}
            sensor.get() = nullptr;
        }
    }
}

builtin_interfaces::msg::Time IMUPublisher::wallClockStamp() {
    return makeWallClockStamp();
}

void IMUPublisher::onIMU1(carla::SharedPtr<carla::sensor::SensorData> data) {
    publishIMU(data, "imu_link", 1);
}

void IMUPublisher::onIMU2(carla::SharedPtr<carla::sensor::SensorData> data) {
    publishIMU(data, "imu_link_2", 2);
}

void IMUPublisher::publishIMU(carla::SharedPtr<carla::sensor::SensorData> data,
                               const std::string& frame_id, int idx) {
    auto imu_data = std::dynamic_pointer_cast<carla::sensor::data::IMUMeasurement>(data);
    if (!imu_data) return;

    constexpr double LO = -99.9;
    constexpr double HI =  99.9;

    auto acc_raw = imu_data->GetAccelerometer();
    auto gyr_raw = imu_data->GetGyroscope();

    double ax = std::clamp(static_cast<double>(acc_raw.x), LO, HI);
    double ay = std::clamp(static_cast<double>(acc_raw.y), LO, HI);
    double az = std::clamp(static_cast<double>(acc_raw.z), LO, HI);
    double gx = std::clamp(static_cast<double>(gyr_raw.x), LO, HI);
    double gy = std::clamp(static_cast<double>(gyr_raw.y), LO, HI);
    double gz = std::clamp(static_cast<double>(gyr_raw.z), LO, HI);

    if (idx == 1) {
        accelerometer_ = {ax, ay, az};
        gyroscope_     = {gx, gy, gz};
        compass_       = imu_data->GetCompass() * 180.0 / M_PI;
    }

    if (!node_) {
        std::cout << "[IMU/" << idx << "] acc=(" << ax << "," << ay << "," << az
                  << ") gyr=(" << gx << "," << gy << "," << gz << ")" << std::endl;
        return;
    }

    auto stamp = wallClockStamp();
    auto msg = sensor_msgs::msg::Imu();
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;

    // CARLA→ROS coordinate conversion
    // Polar vectors (acceleration): (x, -y, z)
    // Axial/pseudo vectors (angular vel): (-x, y, -z)
    msg.linear_acceleration.x  =  ax;
    msg.linear_acceleration.y  = -ay;   // Y flip
    msg.linear_acceleration.z  =  az;
    msg.angular_velocity.x     = -gx;   // Pseudo-vector: x inverted
    msg.angular_velocity.y     =  gy;   // Pseudo-vector: y same
    msg.angular_velocity.z     = -gz;   // Pseudo-vector: z inverted

    // Covariance unknown → mark with -1
    msg.orientation_covariance[0]          = -1.0;
    msg.angular_velocity_covariance[0]     = -1.0;
    msg.linear_acceleration_covariance[0]  = -1.0;

    auto& pub = (idx == 1) ? pub1_ : pub2_;
    pub->publish(msg);
}

}  // namespace carla_sensor_publisher
