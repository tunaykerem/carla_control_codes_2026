#include "ros2_publisher_gcc/ouster_lidar_publisher.hpp"
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

void OusterLidarPublisher::processAndPublish(const float* raw_pts, size_t n_total, double timestamp) {
    // 1. Parse + Y-flip + organize
    std::vector<float> pts(n_total * 4);
    for (size_t i = 0; i < n_total; i++) {
        pts[i*4 + 0] =  raw_pts[i*4 + 0];
        pts[i*4 + 1] = -raw_pts[i*4 + 1]; // CARLA->ROS flip
        pts[i*4 + 2] =  raw_pts[i*4 + 2];
        pts[i*4 + 3] =  raw_pts[i*4 + 3];
    }

    size_t n_per_ch = n_total / NUM_CHANNELS;
    size_t used = n_per_ch * NUM_CHANNELS;

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

    // 2. Mean azimuth -> sector index
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

    // 3. Write sector slot
    sector_grid_[sec_idx] = organized;
    sector_stamp_[sec_idx] = timestamp;

    // 4. Collect fresh sectors
    double STALE_S = 0.20;
    double oldest_allowed = timestamp - STALE_S;
    std::vector<const std::vector<float>*> fresh_sectors;
    for (auto& [idx, pts_vec] : sector_grid_) {
        if (sector_stamp_.count(idx) && sector_stamp_[idx] >= oldest_allowed) {
            fresh_sectors.push_back(&pts_vec);
        }
    }
    if (fresh_sectors.empty()) return;

    size_t total_fresh = 0;
    for (auto* s : fresh_sectors) total_fresh += s->size() / 4;

    size_t n_per_ch_full = total_fresh / NUM_CHANNELS;
    if (n_per_ch_full == 0) return;
    size_t used_full = n_per_ch_full * NUM_CHANNELS;

    // 5. Build 48-byte structured cloud
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
            std::memcpy(p + 0, &x, 4);
            std::memcpy(p + 4, &y, 4);
            std::memcpy(p + 8, &z, 4);
            std::memcpy(p + 16, &intensity, 4);

            uint16_t ring = static_cast<uint16_t>(pt_idx % NUM_CHANNELS);
            std::memcpy(p + 26, &ring, 2);

            float refl_f = std::isnan(intensity) ? 0.0f : intensity;
            uint16_t refl = static_cast<uint16_t>(std::min(65535.0f, std::max(0.0f, refl_f * 257.0f)));
            std::memcpy(p + 24, &refl, 2);

            float range_m = std::sqrt(x*x + y*y + z*z);
            uint32_t range_mm = static_cast<uint32_t>(range_m * 1000.0f);
            std::memcpy(p + 32, &range_mm, 4);

            uint32_t period_ns = 50000000u;
            uint32_t t_val = static_cast<uint32_t>((pt_idx / NUM_CHANNELS) *
                             (period_ns / static_cast<uint32_t>(n_per_ch_full)));
            std::memcpy(p + 20, &t_val, 4);
        }
    }

    // 6. Build and publish PointCloud2 message
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
}

} // namespace carla_sensor_bridge
