#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "ros2_publisher_gcc/tcp_receiver.hpp"

namespace carla_sensor_bridge {

class OusterLidarPublisher {
public:
    static constexpr const char* TOPIC    = "/ouster/points";
    static constexpr const char* FRAME_ID = "ouster";
    static constexpr int NUM_CHANNELS     = 64;
    static constexpr int POINT_STEP       = 48; // x,y,z,pad,intensity,t,reflectivity,ring,ambient,pad,range,pad

    OusterLidarPublisher(rclcpp::Node::SharedPtr node);
    ~OusterLidarPublisher();

private:
    void onTcpDataReceived(const PacketHeader& header, const std::vector<uint8_t>& payload);
    void processAndPublish(const float* raw_pts, size_t n_total, double timestamp);
    std::vector<sensor_msgs::msg::PointField> buildFields();
    builtin_interfaces::msg::Time wallClockStamp();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    std::mutex busy_;

    // Reusable byte buffer to avoid per-frame allocation
    std::vector<uint8_t> reusable_buf_;

    std::vector<sensor_msgs::msg::PointField> fields_;
};

} // namespace carla_sensor_bridge
