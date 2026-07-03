#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "ros2_publisher_gcc/tcp_receiver.hpp"

namespace carla_sensor_bridge {

class ZedCameraPublisher {
public:
    static constexpr const char* IMAGE_TOPIC = "/zed2_left_camera/image_raw";
    static constexpr const char* INFO_TOPIC  = "/zed2_left_camera/camera_info";
    static constexpr const char* FRAME_ID    = "zed_left_camera_optical_frame";
    static constexpr int IMAGE_W = 1280;
    static constexpr int IMAGE_H = 720;
    static constexpr double FOV  = 90.0;
    static constexpr double BRIGHTNESS_SCALE = 3.5;

    ZedCameraPublisher(rclcpp::Node::SharedPtr node);
    ~ZedCameraPublisher();

private:
    void onTcpDataReceived(const PacketHeader& header, const std::vector<uint8_t>& payload);
    sensor_msgs::msg::CameraInfo buildCameraInfo();
    builtin_interfaces::msg::Time wallClockStamp();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    sensor_msgs::msg::CameraInfo camera_info_;

    std::mutex busy_;
};

} // namespace carla_sensor_bridge
