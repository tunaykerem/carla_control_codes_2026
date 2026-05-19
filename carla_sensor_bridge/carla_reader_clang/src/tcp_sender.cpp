#include "carla_reader_clang/tcp_sender.hpp"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

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
        if (queue_.size() > 100) {
            queue_.pop(); // Drop oldest if queue is full
        }
        queue_.push(std::move(pkt));
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
                return !queue_.empty() || !running_; 
            });

            if (!running_) break;
            if (queue_.empty()) continue;

            pkt = std::move(queue_.front());
            queue_.pop();
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
