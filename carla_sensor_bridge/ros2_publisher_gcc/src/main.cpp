#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <csignal>
#include <thread>

#include "ros2_publisher_gcc/tcp_receiver.hpp"
#include "ros2_publisher_gcc/ouster_lidar_publisher.hpp"
#include "ros2_publisher_gcc/zed_camera_publisher.hpp"

using namespace carla_sensor_bridge;

void signalHandler(int /*sig*/) {
    std::cout << "\n[ROS 2 Publisher] Shutting down..." << std::endl;
    TCPReceiver::getInstance().shutdown();
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("carla_ros2_bridge");
    std::signal(SIGINT, signalHandler);

    std::cout << "[ROS 2 Publisher] Starting..." << std::endl;

    if (!TCPReceiver::getInstance().init(DEFAULT_TCP_PORT)) {
        std::cerr << "[ERROR] Failed to start TCP Receiver." << std::endl;
        return 1;
    }

    // Initialize Publishers
    OusterLidarPublisher ouster(node);
    ZedCameraPublisher zed(node);

    std::cout << "\n[ROS 2 Publisher] Ready and listening for CARLA bridge connections.\n\n";

    rclcpp::spin(node);

    TCPReceiver::getInstance().shutdown();
    rclcpp::shutdown();
    return 0;
}
