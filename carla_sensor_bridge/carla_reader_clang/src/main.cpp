#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>
#include <chrono>

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/ActorList.h>

#include "carla_reader_clang/tcp_sender.hpp"
#include "carla_reader_clang/ouster_lidar_reader.hpp"
#include "carla_reader_clang/zed_camera_reader.hpp"

using namespace carla_sensor_bridge;

std::atomic<bool> g_stop{false};

void signalHandler(int /*sig*/) {
    g_stop = true;
    TCPSender::getInstance().shutdown();
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    std::string host = "127.0.0.1";
    int port = 2000;

    std::cout << "[CARLA Reader] Connecting to CARLA on " << host << ":" << port << std::endl;

    try {
        carla::client::Client client(host, static_cast<uint16_t>(port));
        client.SetTimeout(carla::time_duration::seconds(10));
        carla::client::World world = client.GetWorld();

        std::cout << "[CARLA Reader] Connected to Map: " << world.GetMap()->GetName() << std::endl;

        // Initialize TCP Sender
        if (!TCPSender::getInstance().init("127.0.0.1", DEFAULT_TCP_PORT)) {
            std::cerr << "[ERROR] Failed to start TCP Sender." << std::endl;
            return 1;
        }

        // --- Araç Bulma / Spawn ---
        // (For simplicity, just getting the first vehicle found)
        auto actors = world.GetActors();
        carla::SharedPtr<carla::client::Actor> vehicle;
        for (const auto& actor : *actors) {
            if (actor->GetTypeId().find("vehicle.") != std::string::npos) {
                vehicle = actor;
                break;
            }
        }
        
        if (!vehicle) {
            std::cerr << "[ERROR] No vehicle found in the world." << std::endl;
            return 1;
        }

        std::cout << "[CARLA Reader] Attached to vehicle: " << vehicle->GetId() << std::endl;

        // Start Sensor Readers
        OusterLidarReader ouster(vehicle, world);
        ZedCameraReader zed(vehicle, world);

        std::cout << "\n[CARLA Reader] Running... Press Ctrl+C to exit.\n\n";

        while (!g_stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception: " << e.what() << std::endl;
    }

    std::cout << "[CARLA Reader] Shutting down." << std::endl;
    return 0;
}
