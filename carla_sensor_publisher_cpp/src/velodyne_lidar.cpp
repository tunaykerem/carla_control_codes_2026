// Copyright (c) 2025. MIT License.
// velodyne_lidar.cpp — Velodyne LiDAR fast mode, 20-byte PointCloud2

#include "carla_sensor_publisher/velodyne_lidar.hpp"
#include "carla_sensor_publisher/transforms.hpp"

#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorBlueprint.h>

#include <cmath>
#include <cstring>
#include <chrono>

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

VelodyneLidar::VelodyneLidar(carla::SharedPtr<carla::client::Actor> parent,
                             rclcpp::Node::SharedPtr node)
    : node_(node) {

    auto world = parent->GetWorld();
    auto bp_lib = world.GetBlueprintLibrary();
    auto bp = bp_lib->Find("sensor.lidar.ray_cast");

    // DÜZELTME: Const olan blueprint'in değiştirilebilir bir kopyasını oluşturuyoruz
    carla::client::ActorBlueprint mutable_bp = *bp;
    mutable_bp.SetAttribute("channels",          "16");
    mutable_bp.SetAttribute("range",             "100");
    mutable_bp.SetAttribute("points_per_second", "300000");
    mutable_bp.SetAttribute("rotation_frequency","10");
    mutable_bp.SetAttribute("upper_fov",         "15.5");
    mutable_bp.SetAttribute("lower_fov",         "-45.0");
    mutable_bp.SetAttribute("horizontal_fov",    "360");
    mutable_bp.SetAttribute("atmosphere_attenuation_rate", "0.0");
    mutable_bp.SetAttribute("sensor_tick",       "0.1");    // 10 Hz

    auto tf = urdfToCarlaTransform(getTransforms().at("velodyne"));
    // DÜZELTME: SpawnActor'a kopyaladığımız mutable_bp nesnesini veriyoruz
    sensor_ = world.SpawnActor(mutable_bp, tf, parent.get(),
                               carla::rpc::AttachmentType::Rigid);

    if (node) {
        auto qos = rclcpp::QoS(5)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .history(rclcpp::HistoryPolicy::KeepLast);
        pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC, qos);
        fields_ = buildFields();
    }

    // DÜZELTME: boost:: yerine standart std::dynamic_pointer_cast kullanıyoruz
    auto sensor_ptr = std::dynamic_pointer_cast<carla::client::Sensor>(sensor_);
    if (sensor_ptr) {
        sensor_ptr->Listen([this](auto data) {
            this->onSensorData(data);
        });
    }
}

VelodyneLidar::~VelodyneLidar() {
    destroy();
}

void VelodyneLidar::destroy() {
    if (sensor_) {
        try {
            // DÜZELTME: boost:: yerine standart std::dynamic_pointer_cast kullanıyoruz
            auto sensor_ptr = std::dynamic_pointer_cast<carla::client::Sensor>(sensor_);
            if (sensor_ptr && sensor_ptr->IsListening()) {
                sensor_ptr->Stop();
            }
            sensor_->Destroy();
        } catch (...) {}
        sensor_ = nullptr;
    }
}

std::vector<sensor_msgs::msg::PointField> VelodyneLidar::buildFields() {
    using PF = sensor_msgs::msg::PointField;
    return {
        PF().set__name("x").set__offset(0).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("y").set__offset(4).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("z").set__offset(8).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("intensity").set__offset(12).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("ring").set__offset(16).set__datatype(PF::UINT8).set__count(1),
    };
}

builtin_interfaces::msg::Time VelodyneLidar::wallClockStamp() {
    return makeWallClockStamp();
}

void VelodyneLidar::onSensorData(carla::SharedPtr<carla::sensor::SensorData> data) {
    if (!pub_) return;

    auto lidar_data = std::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
    if (!lidar_data) return;

    size_t n_total = lidar_data->size();
    if (n_total < static_cast<size_t>(NUM_CHANNELS)) return;

    const auto* begin = lidar_data->begin();
    const float* raw = reinterpret_cast<const float*>(begin);

    // Copy + Y-flip
    std::vector<float> pts(n_total * 4);
    for (size_t i = 0; i < n_total; i++) {
        pts[i*4 + 0] =  raw[i*4 + 0];
        pts[i*4 + 1] = -raw[i*4 + 1];  // CARLA→ROS
        pts[i*4 + 2] =  raw[i*4 + 2];
        pts[i*4 + 3] =  raw[i*4 + 3];
    }

    size_t n_per_ch = n_total / NUM_CHANNELS;
    size_t used = n_per_ch * NUM_CHANNELS;

    // Channel-major → azimuth-major reorganization
    std::vector<float> organized(used * 4);
    for (size_t az = 0; az < n_per_ch; az++) {
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            size_t src_idx = ch * n_per_ch + az;
            size_t dst_idx = az * NUM_CHANNELS + ch;
            if (src_idx < n_total) {
                organized[dst_idx*4 + 0] = pts[src_idx*4 + 0];
                organized[dst_idx*4 + 1] = pts[src_idx*4 + 1];
                organized[dst_idx*4 + 2] = pts[src_idx*4 + 2];
                organized[dst_idx*4 + 3] = pts[src_idx*4 + 3];
            }
        }
    }

    // Build 20-byte buffer: [x(4) y(4) z(4) intensity(4) ring(1)] + 3 padding = 20
    std::vector<uint8_t> buf(used * POINT_STEP, 0);
    for (size_t i = 0; i < used; i++) {
        uint8_t* p = buf.data() + i * POINT_STEP;
        float x = organized[i*4 + 0];
        float y = organized[i*4 + 1];
        float z = organized[i*4 + 2];
        float intensity = organized[i*4 + 3];
        uint8_t ring = static_cast<uint8_t>(i % NUM_CHANNELS);

        std::memcpy(p + 0,  &x, 4);
        std::memcpy(p + 4,  &y, 4);
        std::memcpy(p + 8,  &z, 4);
        std::memcpy(p + 12, &intensity, 4);
        p[16] = ring;
    }

    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.frame_id = FRAME_ID;
    msg.header.stamp = wallClockStamp();
    msg.height = static_cast<uint32_t>(n_per_ch);
    msg.width  = static_cast<uint32_t>(NUM_CHANNELS);
    msg.is_dense = true;
    msg.is_bigendian = false;
    msg.point_step = POINT_STEP;
    msg.row_step = POINT_STEP * NUM_CHANNELS;
    msg.fields = fields_;
    msg.data = std::move(buf);

    pub_->publish(msg);
}

}  // namespace carla_sensor_publisher
