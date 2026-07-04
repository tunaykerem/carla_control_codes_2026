#include "ros2_publisher_gcc/ouster_lidar_publisher.hpp"
#include "profiling.hpp"
#include <cmath>
#include <cstring>
#include <chrono>
#include <iostream>

namespace carla_sensor_bridge {

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

OusterLidarPublisher::OusterLidarPublisher(rclcpp::Node::SharedPtr node) : node_(node) {
    auto qos = rclcpp::QoS(1)
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .history(rclcpp::HistoryPolicy::KeepLast);
    
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC, qos);
    fields_ = buildFields();

    // Pre-allocate reusable buffer (will be resized if needed)
    reusable_buf_.reserve(NUM_CHANNELS * 512 * POINT_STEP);

    // Register callback with the TCP Receiver
    TCPReceiver::getInstance().registerCallback(SensorType::OUSTER_LIDAR, 
        [this](const PacketHeader& header, const std::vector<uint8_t>& payload) {
            this->onTcpDataReceived(header, payload);
        });
        
    std::cout << "[ROS 2 Publisher] Registered Ouster LiDAR Publisher on topic " << TOPIC << std::endl;
}

OusterLidarPublisher::~OusterLidarPublisher() {}

std::vector<sensor_msgs::msg::PointField> OusterLidarPublisher::buildFields() {
    using PF = sensor_msgs::msg::PointField;
    return {
        PF().set__name("x").set__offset(0).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("y").set__offset(4).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("z").set__offset(8).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("intensity").set__offset(16).set__datatype(PF::FLOAT32).set__count(1),
        PF().set__name("t").set__offset(20).set__datatype(PF::UINT32).set__count(1),
        PF().set__name("reflectivity").set__offset(24).set__datatype(PF::UINT16).set__count(1),
        PF().set__name("ring").set__offset(26).set__datatype(PF::UINT16).set__count(1),
        PF().set__name("ambient").set__offset(28).set__datatype(PF::UINT16).set__count(1),
        PF().set__name("range").set__offset(32).set__datatype(PF::UINT32).set__count(1),
    };
}

builtin_interfaces::msg::Time OusterLidarPublisher::wallClockStamp() {
    return makeWallClockStamp();
}

void OusterLidarPublisher::onTcpDataReceived(const PacketHeader& header, const std::vector<uint8_t>& payload) {
    if (!pub_) return;

    std::unique_lock<std::mutex> lk(busy_, std::try_to_lock);
    if (!lk.owns_lock()) return;

    size_t n_total = payload.size() / (4 * sizeof(float)); // x, y, z, intensity
    if (n_total < static_cast<size_t>(NUM_CHANNELS)) return;

    const float* raw_pts = reinterpret_cast<const float*>(payload.data());
    processAndPublish(raw_pts, n_total, header.timestamp);
}

void OusterLidarPublisher::processAndPublish(const float* raw_pts, size_t n_total, double /*timestamp*/) {
    PROF_PUB_BEGIN("OUSTER");

    // Direct publish: each callback's partial scan is published immediately.
    // No sector accumulation — downstream systems handle partial scans natively.

    size_t n_per_ch = n_total / NUM_CHANNELS;
    size_t used = n_per_ch * NUM_CHANNELS;
    if (used == 0) return;

    // Resize reusable buffer (avoids alloc if capacity is already sufficient)
    size_t buf_size = used * POINT_STEP;
    reusable_buf_.resize(buf_size);
    std::memset(reusable_buf_.data(), 0, buf_size);

    // Time-step for the 't' field (motion compensation)
    uint32_t period_ns = 50000000u; // 50ms = 1/20Hz
    uint32_t t_step = (n_per_ch > 0) ? (period_ns / static_cast<uint32_t>(n_per_ch)) : 0;

    // Single-pass: reorganize channel-first -> azimuth-first, Y-flip, build 48-byte struct
    for (size_t az = 0; az < n_per_ch; az++) {
        uint32_t t_val = static_cast<uint32_t>(az * t_step);
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            size_t src_idx = ch * n_per_ch + az;
            size_t dst_idx = az * NUM_CHANNELS + ch;

            if (src_idx >= n_total) continue;

            float x = raw_pts[src_idx * 4 + 0];
            float y = -raw_pts[src_idx * 4 + 1]; // CARLA->ROS Y-flip
            float z = raw_pts[src_idx * 4 + 2];
            float intensity = raw_pts[src_idx * 4 + 3];

            uint8_t* p = reusable_buf_.data() + dst_idx * POINT_STEP;

            // x, y, z (offset 0, 4, 8)
            std::memcpy(p + 0, &x, 4);
            std::memcpy(p + 4, &y, 4);
            std::memcpy(p + 8, &z, 4);
            // intensity (offset 16)
            std::memcpy(p + 16, &intensity, 4);
            // t (offset 20)
            std::memcpy(p + 20, &t_val, 4);
            // reflectivity (offset 24) — derived from intensity
            float refl_f = std::isnan(intensity) ? 0.0f : intensity;
            uint16_t refl = static_cast<uint16_t>(std::min(65535.0f, std::max(0.0f, refl_f * 257.0f)));
            std::memcpy(p + 24, &refl, 2);
            // ring (offset 26)
            uint16_t ring = static_cast<uint16_t>(ch);
            std::memcpy(p + 26, &ring, 2);
            // range (offset 32)
            float range_m = std::sqrt(x * x + y * y + z * z);
            uint32_t range_mm = static_cast<uint32_t>(range_m * 1000.0f);
            std::memcpy(p + 32, &range_mm, 4);
        }
    }

    PROF_PUB_T4();

    // Build and publish PointCloud2 message
    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.frame_id = FRAME_ID;
    msg.header.stamp = wallClockStamp();
    msg.height = static_cast<uint32_t>(NUM_CHANNELS);
    msg.width  = static_cast<uint32_t>(n_per_ch);
    msg.is_dense = false;
    msg.is_bigendian = false;
    msg.point_step = POINT_STEP;
    msg.row_step = POINT_STEP * n_per_ch;
    msg.fields = fields_;
    msg.data.assign(reusable_buf_.begin(), reusable_buf_.begin() + static_cast<long>(buf_size));

    pub_->publish(msg);

    PROF_PUB_END("OUSTER");
}

} // namespace carla_sensor_bridge
