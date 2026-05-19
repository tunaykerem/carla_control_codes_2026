// Copyright (c) 2025. MIT License.
// zed_camera.hpp — ZED 2 stereo camera
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <carla/client/Sensor.h>
#include <carla/client/World.h>

namespace carla_sensor_publisher {

class ZedCamera {
public:
    static constexpr int IMAGE_W = 1280;
    static constexpr int IMAGE_H = 720;
    static constexpr double FOV  = 90.0;

    ZedCamera(carla::SharedPtr<carla::client::Actor> parent,
              rclcpp::Node::SharedPtr node = nullptr);
    ~ZedCamera();
    void destroy();

private:
    void onLeftImage(carla::SharedPtr<carla::sensor::SensorData> data);
    void onRightImage(carla::SharedPtr<carla::sensor::SensorData> data);
    void publishImage(carla::SharedPtr<carla::sensor::SensorData> data,
                      const std::string& side);
    sensor_msgs::msg::CameraInfo buildCameraInfo(const std::string& frame_id,
                                                  const std::string& side);
    builtin_interfaces::msg::Time wallClockStamp();

    carla::SharedPtr<carla::client::Actor> sensor_left_;
    carla::SharedPtr<carla::client::Actor> sensor_right_;
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;

    sensor_msgs::msg::CameraInfo ci_left_;
    sensor_msgs::msg::CameraInfo ci_right_;

    std::mutex busy_left_;
    std::mutex busy_right_;
};

}  // namespace carla_sensor_publisher
