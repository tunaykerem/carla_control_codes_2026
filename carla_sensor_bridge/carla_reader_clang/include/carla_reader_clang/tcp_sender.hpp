#pragma once

#include "bridge_types.hpp"
#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>

namespace carla_sensor_bridge {

class TCPSender {
public:
    static TCPSender& getInstance() {
        static TCPSender instance;
        return instance;
    }

    bool init(const std::string& ip = "127.0.0.1", int port = DEFAULT_TCP_PORT);
    void shutdown();

    // Kuyruğa asenkron veri ekle (CARLA callback'lerini bloklamamak için)
    // LiDAR verileri yüksek öncelikli kuyruğa, kamera verileri düşük öncelikli kuyruğa eklenir.
    void send(SensorType type, double timestamp, const uint8_t* data, uint32_t size);

private:
    TCPSender();
    ~TCPSender();

    void senderThreadFunc();
    bool connectToServer();

    std::string server_ip_;
    int server_port_;
    int socket_fd_;
    std::atomic<bool> running_;
    std::atomic<bool> connected_;

    struct Packet {
        PacketHeader header;
        std::vector<uint8_t> payload;
    };

    // Dual priority queues: LiDAR (high) vs Camera (low)
    std::queue<Packet> hi_queue_;  // Ouster LiDAR
    std::queue<Packet> lo_queue_;  // ZED Camera & others
    int lidar_sent_since_cam_ = 0; // Fair scheduling counter
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread sender_thread_;
};

} // namespace carla_sensor_bridge
