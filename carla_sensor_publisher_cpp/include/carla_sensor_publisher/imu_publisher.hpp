// Copyright (c) 2025. MIT License.
// imu_publisher.hpp — Dual IMU sensor publisher
#pragma once

#include <memory>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <carla/client/Sensor.h>
#include <carla/client/World.h>

namespace carla_sensor_publisher {

class IMUPublisher {
public:
    IMUPublisher(carla::SharedPtr<carla::client::Actor> parent,
                 rclcpp::Node::SharedPtr node = nullptr);
    ~IMUPublisher();
    void destroy();

    // HUD access
    std::tuple<double,double,double> getAccelerometer() const { return accelerometer_; }
    std::tuple<double,double,double> getGyroscope() const { return gyroscope_; }
    double getCompass() const { return compass_; }

private:
    void onIMU1(carla::SharedPtr<carla::sensor::SensorData> data);
    void onIMU2(carla::SharedPtr<carla::sensor::SensorData> data);
    void publishIMU(carla::SharedPtr<carla::sensor::SensorData> data,
                    const std::string& frame_id, int idx);
    builtin_interfaces::msg::Time wallClockStamp();

    carla::SharedPtr<carla::client::Actor> sensor1_;
    carla::SharedPtr<carla::client::Actor> sensor2_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub2_;

    // HUD values
    std::tuple<double,double,double> accelerometer_{0,0,0};
    std::tuple<double,double,double> gyroscope_{0,0,0};
    double compass_ = 0.0;
};

}  // namespace carla_sensor_publisher
