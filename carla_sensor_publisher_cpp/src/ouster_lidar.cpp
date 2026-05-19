// Copyright (c) 2025. MIT License.
// ouster_lidar.cpp — Ouster OS0-64 LiDAR with sector accumulation
//
// Sector accumulation architecture (from Python):
//   1. Each callback writes incoming slice to its azimuth sector slot
//   2. Each callback merges fresh sectors and publishes full 360° cloud
//   3. Stale sectors (>STALE_S) are filtered by CARLA timestamp

#include "carla_sensor_publisher/ouster_lidar.hpp"
#include "carla_sensor_publisher/transforms.hpp"

#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorBlueprint.h>

#include <cmath>
#include <cstring>
#include <chrono>
#include <iostream>

namespace carla_sensor_publisher {

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

OusterLidar::OusterLidar(carla::SharedPtr<carla::client::Actor> parent,
                         rclcpp::Node::SharedPtr node)
    : node_(node) {

    auto world = parent->GetWorld();
    auto bp_lib = world.GetBlueprintLibrary();
    auto bp = bp_lib->Find("sensor.lidar.ray_cast");

    // DÜZELTME: Const olan bp nesnesinin değiştirilebilir yerel bir kopyasını üretiyoruz
    carla::client::ActorBlueprint mutable_bp = *bp;

    // Nitelikleri artık kilitsiz olan mutable_bp üzerinden set ediyoruz
    mutable_bp.SetAttribute("channels",          "64");
    mutable_bp.SetAttribute("range",             "50");
    mutable_bp.SetAttribute("points_per_second", "1310720");
    mutable_bp.SetAttribute("rotation_frequency","20");
    mutable_bp.SetAttribute("upper_fov",         "45.0");
    mutable_bp.SetAttribute("lower_fov",         "-45.0");
    mutable_bp.SetAttribute("horizontal_fov",    "360");
    mutable_bp.SetAttribute("atmosphere_attenuation_rate", "0.0");
    mutable_bp.SetAttribute("sensor_tick",       "0.05");   // 20 Hz callback

    // FIX 1: Disable dropoff (CARLA defaults drop ~45% of points)
    mutable_bp.SetAttribute("dropoff_general_rate",    "0.0");
    // FIX 2: Disable intensity-based drop-off
    mutable_bp.SetAttribute("dropoff_zero_intensity",  "0.0");
    mutable_bp.SetAttribute("dropoff_intensity_limit", "1.0");
    // Noise
    mutable_bp.SetAttribute("noise_stddev",            "0.03");

    auto tf = urdfToCarlaTransform(getTransforms().at("ouster"));
    // DÜZELTME: SpawnActor fonksiyonuna mutable_bp nesnemizi gönderiyoruz
    sensor_ = world.SpawnActor(mutable_bp, tf, parent.get(),
                               carla::rpc::AttachmentType::Rigid);

    if (node) {
        auto qos = rclcpp::QoS(1)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .history(rclcpp::HistoryPolicy::KeepLast);
        pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC, qos);
        fields_ = buildFields();
    }

    // Register CARLA callback (Burası zaten std:: olarak kalabilir, doğru yapmışsın)
    auto sensor_ptr = std::dynamic_pointer_cast<carla::client::Sensor>(sensor_);
    if (sensor_ptr) {
        sensor_ptr->Listen([this](auto data) {
            this->onSensorData(data);
        });
    }
}

OusterLidar::~OusterLidar() {
    destroy();
}

void OusterLidar::destroy() {
    if (sensor_) {
        try {
            auto sensor_ptr = std::dynamic_pointer_cast<carla::client::Sensor>(sensor_);
            if (sensor_ptr && sensor_ptr->IsListening()) {
                sensor_ptr->Stop();
            }
            sensor_->Destroy();
        } catch (...) {}
        sensor_ = nullptr;
    }
}

std::vector<sensor_msgs::msg::PointField> OusterLidar::buildFields() {
    using PF = sensor_msgs::msg::PointField;
    return {
        PF().set__name("x").set__offset(0).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("y").set__offset(4).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("z").set__offset(8).set__datatype(PF::FLOAT32).set__count(1),
        // offset 12: 4-byte padding (PCL_ADD_POINT4D)
        PF().set__name("intensity").set__offset(16).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("t").set__offset(20).set__datatype(PF::UINT32).set__count(1),
        PF().set__name("reflectivity").set__offset(24).set__datatype(PF::UINT16).set__count(1),
        PF().set__name("ring").set__offset(26).set__datatype(PF::UINT16).set__count(1),
        PF().set__name("ambient").set__offset(28).set__datatype(PF::UINT16).set__count(1),
        PF().set__name("range").set__offset(32).set__datatype(PF::UINT32).set__count(1),
    };
}

builtin_interfaces::msg::Time OusterLidar::wallClockStamp() {
    return makeWallClockStamp();
}

void OusterLidar::onSensorData(carla::SharedPtr<carla::sensor::SensorData> data) {
    if (!pub_) return;

    // Non-blocking: if busy, drop this callback
    std::unique_lock<std::mutex> lk(busy_, std::try_to_lock);
    if (!lk.owns_lock()) {
        prof_dropped_++;
        return;
    }

    auto lidar_data = std::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
    if (!lidar_data) return;

    size_t n_total = lidar_data->size();
    if (n_total < static_cast<size_t>(NUM_CHANNELS)) return;

    // Get raw float pointer: each point = {x, y, z, intensity}
    const auto* begin = lidar_data->begin();
    // LidarDetection has {point.x, point.y, point.z, intensity} = 4 floats
    const float* raw_pts = reinterpret_cast<const float*>(begin);
    double timestamp = lidar_data->GetTimestamp();

    processAndPublish(raw_pts, n_total, timestamp);
}

void OusterLidar::processAndPublish(const float* raw_pts, size_t n_total,
                                     double timestamp) {
    // ── 1. Parse + Y-flip + organize ──────────────────────────────────
    // Copy raw data and flip Y axis (CARLA→ROS)
    std::vector<float> pts(n_total * 4);
    for (size_t i = 0; i < n_total; i++) {
        pts[i*4 + 0] =  raw_pts[i*4 + 0];       // x
        pts[i*4 + 1] = -raw_pts[i*4 + 1];       // y: CARLA→ROS flip
        pts[i*4 + 2] =  raw_pts[i*4 + 2];       // z
        pts[i*4 + 3] =  raw_pts[i*4 + 3];       // intensity
    }

    size_t n_per_ch = n_total / NUM_CHANNELS;
    size_t used = n_per_ch * NUM_CHANNELS;

    // Channel-major → azimuth-major reorganization
    // organized[az][ch] = pts[ch * n_per_ch + az]
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

    // ── 2. Mean azimuth → sector index ────────────────────────────────
    double sum_az = 0.0;
    size_t count_az = 0;
    for (size_t i = 0; i < used; i++) {
        float x = organized[i*4 + 0];
        float y = organized[i*4 + 1];
        if (x != 0.0f || y != 0.0f) {
            sum_az += std::atan2(y, x);
            count_az++;
        }
    }
    if (count_az == 0) return;
    double mean_az = sum_az / count_az;
    int sec_idx = static_cast<int>((mean_az + M_PI) / (2.0 * M_PI) * NUM_SECTORS) % NUM_SECTORS;
    if (sec_idx < 0) sec_idx += NUM_SECTORS;

    // ── 3. Write sector slot ──────────────────────────────────────────
    sector_grid_[sec_idx] = organized;
    sector_stamp_[sec_idx] = timestamp;

    // ── 4. Collect fresh sectors ──────────────────────────────────────
    double oldest_allowed = timestamp - STALE_S;
    std::vector<const std::vector<float>*> fresh_sectors;
    for (auto& [idx, pts_vec] : sector_grid_) {
        if (sector_stamp_.count(idx) && sector_stamp_[idx] >= oldest_allowed) {
            fresh_sectors.push_back(&pts_vec);
        }
    }
    if (fresh_sectors.empty()) return;

    // Concatenate all fresh sector points
    size_t total_fresh = 0;
    for (auto* s : fresh_sectors) total_fresh += s->size() / 4;

    // Ensure divisible by NUM_CHANNELS
    size_t n_per_ch_full = total_fresh / NUM_CHANNELS;
    if (n_per_ch_full == 0) return;
    size_t used_full = n_per_ch_full * NUM_CHANNELS;

    // ── 5. Build 48-byte structured cloud ─────────────────────────────
    std::vector<uint8_t> buf(used_full * POINT_STEP, 0);

    size_t pt_idx = 0;
    for (auto* sector_pts : fresh_sectors) {
        size_t n_pts = sector_pts->size() / 4;
        for (size_t i = 0; i < n_pts && pt_idx < used_full; i++, pt_idx++) {
            float x = (*sector_pts)[i*4 + 0];
            float y = (*sector_pts)[i*4 + 1];
            float z = (*sector_pts)[i*4 + 2];
            float intensity = (*sector_pts)[i*4 + 3];

            uint8_t* p = buf.data() + pt_idx * POINT_STEP;

            // x (offset 0)
            std::memcpy(p + 0, &x, 4);
            // y (offset 4)
            std::memcpy(p + 4, &y, 4);
            // z (offset 8)
            std::memcpy(p + 8, &z, 4);
            // padding (offset 12) — zero
            // intensity (offset 16)
            std::memcpy(p + 16, &intensity, 4);

            // ring (offset 26)
            uint16_t ring = static_cast<uint16_t>(pt_idx % NUM_CHANNELS);
            std::memcpy(p + 26, &ring, 2);

            // reflectivity (offset 24)
            float refl_f = std::isnan(intensity) ? 0.0f : intensity;
            uint16_t refl = static_cast<uint16_t>(std::min(65535.0f, std::max(0.0f, refl_f * 257.0f)));
            std::memcpy(p + 24, &refl, 2);

            // range (offset 32)
            float range_m = std::sqrt(x*x + y*y + z*z);
            uint32_t range_mm = static_cast<uint32_t>(range_m * 1000.0f);
            std::memcpy(p + 32, &range_mm, 4);

            // t (offset 20)
            uint32_t period_ns = 50000000u;  // 50ms
            uint32_t t_val = static_cast<uint32_t>((pt_idx / NUM_CHANNELS) *
                             (period_ns / static_cast<uint32_t>(n_per_ch_full)));
            std::memcpy(p + 20, &t_val, 4);
        }
    }

    // ── 6. Build and publish PointCloud2 message ──────────────────────
    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.frame_id = FRAME_ID;
    msg.header.stamp = wallClockStamp();
    msg.height = static_cast<uint32_t>(n_per_ch_full);
    msg.width  = static_cast<uint32_t>(NUM_CHANNELS);
    msg.is_dense = false;
    msg.is_bigendian = false;
    msg.point_step = POINT_STEP;
    msg.row_step = POINT_STEP * NUM_CHANNELS;
    msg.fields = fields_;
    msg.data = std::move(buf);

    pub_->publish(msg);

    // ── Profiling ─────────────────────────────────────────────────────
    int cnt = ++prof_count_;
    if (cnt % PROF_EVERY == 0) {
        std::cout << "[OUSTER] Published " << cnt << " clouds, "
                  << used_full << " points, "
                  << fresh_sectors.size() << "/" << NUM_SECTORS << " sectors, "
                  << "dropped=" << prof_dropped_.load() << std::endl;
        prof_dropped_ = 0;
    }
}

}  // namespace carla_sensor_publisher
