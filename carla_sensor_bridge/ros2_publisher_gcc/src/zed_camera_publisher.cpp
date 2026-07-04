// Copyright (c) 2025. MIT License.
// zed_camera_publisher.cpp — ZED mono camera: TCP JPEG → BGR8 Image + CameraInfo
//
// Receives JPEG-compressed BGR frames from carla_reader_clang via TCP bridge.
// Brightness scaling is done on the reader side before compression.
// This publisher just decodes JPEG and publishes as ROS 2 Image.

#include "ros2_publisher_gcc/zed_camera_publisher.hpp"
#include "profiling.hpp"

#include <cmath>
#include <cstring>
#include <chrono>
#include <iostream>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

namespace carla_sensor_bridge {

// ── Wall-clock timestamp ─────────────────────────────────────────────────
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

ZedCameraPublisher::ZedCameraPublisher(rclcpp::Node::SharedPtr node)
    : node_(node) {

    auto qos = rclcpp::QoS(1)
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .history(rclcpp::HistoryPolicy::KeepLast);

    pub_img_  = node_->create_publisher<sensor_msgs::msg::Image>(IMAGE_TOPIC, qos);
    pub_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(INFO_TOPIC, qos);

    camera_info_ = buildCameraInfo();

    // Register callbacks for left camera data from TCP bridge
    TCPReceiver::getInstance().registerCallback(SensorType::ZED_LEFT_IMAGE,
        [this](const PacketHeader& header, const std::vector<uint8_t>& payload) {
            this->onTcpDataReceived(header, payload);
        });

    std::cout << "[ROS 2 Publisher] Registered ZED Camera Publisher on topics "
              << IMAGE_TOPIC << ", " << INFO_TOPIC
              << " [JPEG decode mode]" << std::endl;
}

ZedCameraPublisher::~ZedCameraPublisher() {}

sensor_msgs::msg::CameraInfo ZedCameraPublisher::buildCameraInfo() {
    // Pinhole model: fx = W / (2 * tan(FOV/2))
    double fx = static_cast<double>(IMAGE_W) / (2.0 * std::tan(FOV * M_PI / 360.0));
    double fy = fx;
    double cx = static_cast<double>(IMAGE_W) / 2.0;
    double cy = static_cast<double>(IMAGE_H) / 2.0;

    sensor_msgs::msg::CameraInfo ci;
    ci.header.frame_id = FRAME_ID;
    ci.width  = IMAGE_W;
    ci.height = IMAGE_H;
    ci.distortion_model = "plumb_bob";

    // D: [k1, k2, t1, t2, k3] — no distortion
    ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // K: 3×3 intrinsic matrix (row-major)
    ci.k = {fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0};

    // R: 3×3 rectification (identity for mono)
    ci.r = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};

    // P: 3×4 projection (Tx=0 for left/mono camera)
    ci.p = {fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0};

    return ci;
}

builtin_interfaces::msg::Time ZedCameraPublisher::wallClockStamp() {
    return makeWallClockStamp();
}

void ZedCameraPublisher::onTcpDataReceived(const PacketHeader& /*header*/,
                                            const std::vector<uint8_t>& payload) {
    if (!pub_img_) return;

    PROF_PUB_BEGIN("ZED");

    // Payload format: [width(4B) | height(4B) | JPEG compressed BGR data]
    if (payload.size() < 8) return;

    uint32_t width, height;
    std::memcpy(&width,  payload.data(),     sizeof(uint32_t));
    std::memcpy(&height, payload.data() + 4, sizeof(uint32_t));

    size_t jpeg_size = payload.size() - 8;
    if (jpeg_size == 0) return;

    // ── 1. Decode JPEG → BGR cv::Mat ─────────────────────────────────────
    cv::Mat jpeg_buf(1, static_cast<int>(jpeg_size), CV_8UC1,
                     const_cast<uint8_t*>(payload.data() + 8));
    cv::Mat bgr = cv::imdecode(jpeg_buf, cv::IMREAD_COLOR);

    if (bgr.empty() ||
        bgr.cols != static_cast<int>(width) ||
        bgr.rows != static_cast<int>(height)) {
        return;
    }

    // ── 2. Build ROS 2 Image message (brightness already applied by reader) ─
    auto stamp = wallClockStamp();

    std_msgs::msg::Header header_msg;
    header_msg.frame_id = FRAME_ID;
    header_msg.stamp = stamp;

    auto img_msg = cv_bridge::CvImage(header_msg, "bgr8", bgr).toImageMsg();

    PROF_PUB_T4();

    // ── 3. Publish Image + CameraInfo ────────────────────────────────────
    camera_info_.header.stamp = stamp;
    pub_img_->publish(*img_msg);
    pub_info_->publish(camera_info_);

    PROF_PUB_END("ZED");
}

} // namespace carla_sensor_bridge
