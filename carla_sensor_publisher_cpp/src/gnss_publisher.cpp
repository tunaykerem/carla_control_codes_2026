// Copyright (c) 2025. MIT License.
// gnss_publisher.cpp — Dual GNSS sensor publisher

#include "carla_sensor_publisher/gnss_publisher.hpp"
#include "carla_sensor_publisher/transforms.hpp"

#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorBlueprint.h>

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

GNSSPublisher::GNSSPublisher(carla::SharedPtr<carla::client::Actor> parent,
                             rclcpp::Node::SharedPtr node)
    : node_(node) {

    auto world = parent->GetWorld();
    auto bp_lib = world.GetBlueprintLibrary();
    auto bp = bp_lib->Find("sensor.other.gnss");
    carla::client::ActorBlueprint mutable_bp = *bp;
    mutable_bp.SetAttribute("sensor_tick", "0.1");  // 10 Hz

    auto tf_front = urdfToCarlaTransform(getTransforms().at("gnss_front_right"));
    auto tf_rear  = urdfToCarlaTransform(getTransforms().at("gnss_rear_right"));

    sensor_front_ = world.SpawnActor(mutable_bp, tf_front, parent.get(),
                                     carla::rpc::AttachmentType::Rigid);
    sensor_rear_  = world.SpawnActor(mutable_bp, tf_rear, parent.get(),
                                     carla::rpc::AttachmentType::Rigid);

    if (node) {
        auto qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .history(rclcpp::HistoryPolicy::KeepLast);
        pub_front_ = node->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/sbg/ros/nav_sat_fix", qos);
        pub_rear_ = node->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/gnss/rear_right/fix", qos);
    }

    auto sf = std::dynamic_pointer_cast<carla::client::Sensor>(sensor_front_);
    if (sf) {
        sf->Listen([this](auto data) { this->onFront(data); });
    }
    auto sr = std::dynamic_pointer_cast<carla::client::Sensor>(sensor_rear_);
    if (sr) {
        sr->Listen([this](auto data) { this->onRear(data); });
    }
}

GNSSPublisher::~GNSSPublisher() {
    destroy();
}

void GNSSPublisher::destroy() {
    for (auto& sensor : {std::ref(sensor_front_), std::ref(sensor_rear_)}) {
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

builtin_interfaces::msg::Time GNSSPublisher::wallClockStamp() {
    return makeWallClockStamp();
}

void GNSSPublisher::onFront(carla::SharedPtr<carla::sensor::SensorData> data) {
    publishGNSS(data, "imu_link", "front");
}

void GNSSPublisher::onRear(carla::SharedPtr<carla::sensor::SensorData> data) {
    publishGNSS(data, "gnss_rear_right", "rear");
}

void GNSSPublisher::publishGNSS(carla::SharedPtr<carla::sensor::SensorData> data,
                                 const std::string& frame_id, const std::string& side) {
    auto gnss = std::dynamic_pointer_cast<carla::sensor::data::GnssMeasurement>(data);
    if (!gnss) return;

    if (side == "front") {
        lat_ = gnss->GetLatitude();
        lon_ = gnss->GetLongitude();
    }

    if (!node_) {
        std::cout << "[GNSS/" << side << "] lat=" << gnss->GetLatitude()
                  << " lon=" << gnss->GetLongitude()
                  << " alt=" << gnss->GetAltitude() << std::endl;
        return;
    }

    auto stamp = wallClockStamp();
    auto msg = sensor_msgs::msg::NavSatFix();
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.status.status  = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    msg.latitude  = gnss->GetLatitude();
    msg.longitude = gnss->GetLongitude();
    msg.altitude  = gnss->GetAltitude();
    msg.position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    auto& pub = (side == "front") ? pub_front_ : pub_rear_;
    pub->publish(msg);
}

}  // namespace carla_sensor_publisher
