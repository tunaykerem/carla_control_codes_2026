#include "carla_reader_clang/tcp_sender.hpp"
#include "profiling.hpp"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <netinet/tcp.h>

namespace carla_sensor_bridge {

TCPSender::TCPSender() : socket_fd_(-1), running_(false), connected_(false) {}

TCPSender::~TCPSender() {
    shutdown();
}

bool TCPSender::init(const std::string& ip, int port) {
    server_ip_ = ip;
    server_port_ = port;
    running_ = true;
    sender_thread_ = std::thread(&TCPSender::senderThreadFunc, this);
    return true;
}

void TCPSender::shutdown() {
    if (running_) {
        running_ = false;
        cv_.notify_all();
        if (sender_thread_.joinable()) {
            sender_thread_.join();
        }
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }
}

void TCPSender::send(SensorType type, double timestamp, const uint8_t* data, uint32_t size) {
    Packet pkt;
    pkt.header.magic = MAGIC_BYTES;
    pkt.header.type = type;
    pkt.header.timestamp = timestamp;
    pkt.header.payload_size = size;

    if (size > 0 && data != nullptr) {
        pkt.payload.assign(data, data + size);
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Separate handling: LiDAR gets priority, camera is drop-if-full
        if (type == SensorType::OUSTER_LIDAR) {
            // LiDAR: keep up to 10 in high-priority queue, drop oldest if exceeded
            if (hi_queue_.size() > 10) {
                hi_queue_.pop();
            }
            hi_queue_.push(std::move(pkt));
        } else {
            // Camera/other: keep only the latest 2 frames
            while (lo_queue_.size() >= 2) {
                lo_queue_.pop();
            }
            lo_queue_.push(std::move(pkt));
        }
        PROF_TCP_QUEUE(hi_queue_.size() + lo_queue_.size(), size);
    }
    cv_.notify_one();
}

bool TCPSender::connectToServer() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        return false;
    }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(server_port_);

    if (inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
        return false;
    }

    if (::connect(socket_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        return false;
    }

    // Set send buffer size to a larger value for LiDAR data (e.g., 8MB)
    int send_buf_size = 8 * 1024 * 1024;
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

    // Disable Nagle's algorithm for lower latency
    int flag = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    return true;
}

void TCPSender::senderThreadFunc() {
    while (running_) {
        if (!connected_) {
            if (connectToServer()) {
                std::cout << "[TCPSender] Connected to ROS 2 bridge server at " 
                          << server_ip_ << ":" << server_port_ << std::endl;
                connected_ = true;
            } else {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
        }

        Packet pkt;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                return !hi_queue_.empty() || !lo_queue_.empty() || !running_; 
            });

            if (!running_) break;

            // Fair scheduling: after 2 consecutive LiDAR sends, force 1 camera send
            bool cam_starved = !lo_queue_.empty() && lidar_sent_since_cam_ >= 2;
            if (cam_starved) {
                pkt = std::move(lo_queue_.front());
                lo_queue_.pop();
                lidar_sent_since_cam_ = 0;
            } else if (!hi_queue_.empty()) {
                pkt = std::move(hi_queue_.front());
                hi_queue_.pop();
                lidar_sent_since_cam_++;
            } else if (!lo_queue_.empty()) {
                pkt = std::move(lo_queue_.front());
                lo_queue_.pop();
                lidar_sent_since_cam_ = 0;
            } else {
                continue;
            }
        }

        // Send header
        ssize_t bytes_sent = ::send(socket_fd_, &pkt.header, sizeof(PacketHeader), MSG_NOSIGNAL);
        if (bytes_sent != sizeof(PacketHeader)) {
            std::cerr << "[TCPSender] Failed to send header, reconnecting..." << std::endl;
            connected_ = false;
            continue;
        }

        // Send payload
        if (pkt.header.payload_size > 0) {
            size_t total_sent = 0;
            const uint8_t* ptr = pkt.payload.data();
            while (total_sent < pkt.header.payload_size) {
                ssize_t sent = ::send(socket_fd_, ptr + total_sent, pkt.header.payload_size - total_sent, MSG_NOSIGNAL);
                if (sent <= 0) {
                    std::cerr << "[TCPSender] Failed to send payload, reconnecting..." << std::endl;
                    connected_ = false;
                    break;
                }
                total_sent += sent;
            }
        }
    }
}

} // namespace carla_sensor_bridge
