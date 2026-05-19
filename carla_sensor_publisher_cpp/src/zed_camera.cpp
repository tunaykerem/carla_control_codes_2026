// Copyright (c) 2025. MIT License.
// zed_camera.cpp — ZED 2 stereo camera with 3.5× brightness boost

#include "carla_sensor_publisher/zed_camera.hpp"
#include "carla_sensor_publisher/transforms.hpp"

#include <carla/sensor/data/Image.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorBlueprint.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <chrono>
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

ZedCamera::ZedCamera(carla::SharedPtr<carla::client::Actor> parent,
                     rclcpp::Node::SharedPtr node)
    : node_(node) {

    auto world = parent->GetWorld();
    auto bp_lib = world.GetBlueprintLibrary();

    auto make_camera_bp = [&](const std::string& tick) {
        auto bp = bp_lib->Find("sensor.camera.rgb");
        carla::client::ActorBlueprint mutable_bp = *bp;
        mutable_bp.SetAttribute("image_size_x", std::to_string(IMAGE_W));
        mutable_bp.SetAttribute("image_size_y", std::to_string(IMAGE_H));
        mutable_bp.SetAttribute("fov",          std::to_string(FOV));
        mutable_bp.SetAttribute("sensor_tick",   tick);
        mutable_bp.SetAttribute("enable_postprocess_effects", "True");
        return mutable_bp;
    };

    auto tf_left  = urdfToCarlaTransform(getTransforms().at("zed_left"));
    auto tf_right = urdfToCarlaTransform(getTransforms().at("zed_right"));

    sensor_left_  = world.SpawnActor(make_camera_bp("0.05"), tf_left, parent.get());
    sensor_right_ = world.SpawnActor(make_camera_bp("0.06"), tf_right, parent.get());

    if (node) {
        auto qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .history(rclcpp::HistoryPolicy::KeepLast);

        pub_left_img_   = node->create_publisher<sensor_msgs::msg::Image>(
            "/zed/zed_node/left/image_rect_color", qos);
        pub_left_info_  = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/zed/left/camera_info", qos);
        pub_right_img_  = node->create_publisher<sensor_msgs::msg::Image>(
            "/zed/right/image_raw", qos);
        pub_right_info_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/zed/right/camera_info", qos);

        ci_left_  = buildCameraInfo("zed_left_camera_optical_frame", "left");
        ci_right_ = buildCameraInfo("zed_right_camera_optical_frame", "right");
    }

    // Register callbacks
    auto left_sensor = std::dynamic_pointer_cast<carla::client::Sensor>(sensor_left_);
    if (left_sensor) {
        left_sensor->Listen([this](auto data) { this->onLeftImage(data); });
    }
    auto right_sensor = std::dynamic_pointer_cast<carla::client::Sensor>(sensor_right_);
    if (right_sensor) {
        right_sensor->Listen([this](auto data) { this->onRightImage(data); });
    }
}

ZedCamera::~ZedCamera() {
    destroy();
}

void ZedCamera::destroy() {
    for (auto& sensor : {std::ref(sensor_left_), std::ref(sensor_right_)}) {
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

sensor_msgs::msg::CameraInfo ZedCamera::buildCameraInfo(const std::string& frame_id,
                                                         const std::string& side) {
    double fx = IMAGE_W / (2.0 * std::tan(FOV * M_PI / 360.0));
    double fy = fx;
    double cx = IMAGE_W / 2.0;
    double cy = IMAGE_H / 2.0;
    double tx = (side == "right") ? -fx * 0.12 : 0.0;

    sensor_msgs::msg::CameraInfo ci;
    ci.width = IMAGE_W;
    ci.height = IMAGE_H;
    ci.distortion_model = "plumb_bob";
    ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    ci.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    ci.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    ci.p = {fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    ci.header.frame_id = frame_id;
    return ci;
}

builtin_interfaces::msg::Time ZedCamera::wallClockStamp() {
    return makeWallClockStamp();
}

void ZedCamera::onLeftImage(carla::SharedPtr<carla::sensor::SensorData> data) {
    std::unique_lock<std::mutex> lk(busy_left_, std::try_to_lock);
    if (!lk.owns_lock()) return;
    publishImage(data, "left");
}

void ZedCamera::onRightImage(carla::SharedPtr<carla::sensor::SensorData> data) {
    std::unique_lock<std::mutex> lk(busy_right_, std::try_to_lock);
    if (!lk.owns_lock()) return;
    publishImage(data, "right");
}

void ZedCamera::publishImage(carla::SharedPtr<carla::sensor::SensorData> data,
                              const std::string& side) {
    if (!node_) return;

    auto image = std::dynamic_pointer_cast<carla::sensor::data::Image>(data);
    if (!image) return;

    uint32_t w = image->GetWidth();
    uint32_t h = image->GetHeight();
    const auto* raw = image->begin();

    // Convert BGRA to cv::Mat
    cv::Mat bgra(h, w, CV_8UC4, const_cast<carla::sensor::data::Color*>(raw));
    cv::Mat out = bgra.clone();

    // 3.5× brightness boost
    out.convertTo(out, -1, 3.5, 0);

    // Convert to ROS Image
    auto stamp = wallClockStamp();
    std::string frame_id = "zed_" + side + "_camera_optical_frame";

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;

    auto img_msg = cv_bridge::CvImage(header, "bgra8", out).toImageMsg();

    // Publish
    auto& ci = (side == "left") ? ci_left_ : ci_right_;
    ci.header.stamp = stamp;

    if (side == "left") {
        pub_left_img_->publish(*img_msg);
        pub_left_info_->publish(ci);
    } else {
        pub_right_img_->publish(*img_msg);
        pub_right_info_->publish(ci);
    }
}

}  // namespace carla_sensor_publisher
