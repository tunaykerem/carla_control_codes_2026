#pragma once

#include "bridge_types.hpp"
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include <map>
#include <mutex>

namespace carla_sensor_bridge {

class TCPReceiver {
public:
    using PacketCallback = std::function<void(const PacketHeader&, const std::vector<uint8_t>&)>;

    static TCPReceiver& getInstance() {
        static TCPReceiver instance;
        return instance;
    }

    bool init(int port = DEFAULT_TCP_PORT);
    void shutdown();

    void registerCallback(SensorType type, PacketCallback callback);

private:
    TCPReceiver();
    ~TCPReceiver();

    void serverThreadFunc();
    void clientThreadFunc(int client_fd);

    int server_port_;
    int server_fd_;
    std::atomic<bool> running_;

    std::thread server_thread_;
    std::vector<std::thread> client_threads_;

    std::mutex cb_mutex_;
    std::map<SensorType, PacketCallback> callbacks_;
};

} // namespace carla_sensor_bridge
