#include "ros2_publisher_gcc/tcp_receiver.hpp"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <poll.h>

namespace carla_sensor_bridge {

TCPReceiver::TCPReceiver() : server_fd_(-1), running_(false) {}

TCPReceiver::~TCPReceiver() {
    shutdown();
}

bool TCPReceiver::init(int port) {
    server_port_ = port;
    
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "[TCPReceiver] Failed to create socket." << std::endl;
        return false;
    }

    int opt = 1;
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cerr << "[TCPReceiver] Failed to setsockopt." << std::endl;
        return false;
    }

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(server_port_);

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "[TCPReceiver] Bind failed on port " << server_port_ << std::endl;
        return false;
    }

    if (listen(server_fd_, 3) < 0) {
        std::cerr << "[TCPReceiver] Listen failed." << std::endl;
        return false;
    }

    running_ = true;
    server_thread_ = std::thread(&TCPReceiver::serverThreadFunc, this);
    
    std::cout << "[TCPReceiver] Listening on port " << server_port_ << std::endl;
    return true;
}

void TCPReceiver::shutdown() {
    if (running_) {
        running_ = false;
        if (server_fd_ >= 0) {
            ::shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
            server_fd_ = -1;
        }
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
        for (auto& t : client_threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
    }
}

void TCPReceiver::registerCallback(SensorType type, PacketCallback callback) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    callbacks_[type] = callback;
}

void TCPReceiver::serverThreadFunc() {
    while (running_) {
        struct pollfd pfd;
        pfd.fd = server_fd_;
        pfd.events = POLLIN;
        
        int ret = poll(&pfd, 1, 1000); // 1 sec timeout
        if (ret > 0 && (pfd.revents & POLLIN)) {
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
            
            if (client_fd >= 0) {
                std::cout << "[TCPReceiver] New client connected." << std::endl;
                
                int recv_buf_size = 8 * 1024 * 1024;
                setsockopt(client_fd, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size));

                client_threads_.emplace_back(&TCPReceiver::clientThreadFunc, this, client_fd);
            }
        }
    }
}

void TCPReceiver::clientThreadFunc(int client_fd) {
    std::vector<uint8_t> payload_buffer;
    
    while (running_) {
        PacketHeader header;
        
        // Read header
        size_t header_received = 0;
        while (header_received < sizeof(PacketHeader)) {
            ssize_t bytes = recv(client_fd, reinterpret_cast<uint8_t*>(&header) + header_received, 
                                 sizeof(PacketHeader) - header_received, 0);
            if (bytes <= 0) {
                std::cout << "[TCPReceiver] Client disconnected." << std::endl;
                close(client_fd);
                return;
            }
            header_received += bytes;
        }

        if (header.magic != MAGIC_BYTES) {
            std::cerr << "[TCPReceiver] Invalid magic bytes! Expected 0xCACA0000, got " << std::hex << header.magic << std::dec << std::endl;
            continue;
        }

        // Read payload
        if (header.payload_size > 0) {
            payload_buffer.resize(header.payload_size);
            size_t payload_received = 0;
            while (payload_received < header.payload_size) {
                ssize_t bytes = recv(client_fd, payload_buffer.data() + payload_received, 
                                     header.payload_size - payload_received, 0);
                if (bytes <= 0) {
                    std::cout << "[TCPReceiver] Client disconnected during payload." << std::endl;
                    close(client_fd);
                    return;
                }
                payload_received += bytes;
            }
        } else {
            payload_buffer.clear();
        }

        // Dispatch to registered callback
        PacketCallback cb = nullptr;
        {
            std::lock_guard<std::mutex> lock(cb_mutex_);
            auto it = callbacks_.find(header.type);
            if (it != callbacks_.end()) {
                cb = it->second;
            }
        }

        if (cb) {
            cb(header, payload_buffer);
        }
    }
    
    close(client_fd);
}

} // namespace carla_sensor_bridge
