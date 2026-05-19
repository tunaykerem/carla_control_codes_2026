// Copyright (c) 2025. MIT License.
//
// MyVehicle Sensor Publisher — C++ ROS 2 Node for CARLA 0.10 / UE5
// ================================================================
//
// Araç üzerindeki sensörlerin tamamını doğru TF konumlarıyla CARLA'ya ekler
// ve ROS 2 topic'lerine yayınlar.
//
// Sensörler (URDF'den alınan TF'ler):
//   - Ouster OS0-64 LiDAR        : /ouster/points
//   - Velodyne LiDAR              : /velodyne/points
//   - ZED Camera (sol/sag)        : /zed/zed_node/left/image_rect_color,
//                                   /zed/right/image_raw
//   - ZED Camera Info             : /zed/left/camera_info, /zed/right/camera_info
//   - IMU 1 (SBG)                 : /sbg/ros/imu/data
//   - IMU 2                       : /imu/imu2/data
//   - GNSS (SBG NavSatFix)        : /sbg/ros/nav_sat_fix
//   - GNSS Arka Sağ               : /gnss/rear_right/fix
//
// Kullanım:
//   ros2 run carla_sensor_publisher carla_sensor_node \
//       --ros-args -p host:=127.0.0.1 -p port:=2000 -p sync:=false

#include <csignal>
#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/ActorList.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/rpc/EpisodeSettings.h>

#include "carla_sensor_publisher/sensor_manager.hpp"
#include "carla_sensor_publisher/vehicle_controller.hpp"

using namespace carla_sensor_publisher;

static std::atomic<bool> g_stop{false};

static void signalHandler(int /*sig*/) {
    std::cout << "\n[INFO] Durduruluyor..." << std::endl;
    g_stop = true;
}

// ── Araç bulma / spawn ────────────────────────────────────────────────────
static carla::SharedPtr<carla::client::Actor>
findOrSpawnVehicle(carla::client::World& world,
                   const std::string& filter_pattern,
                   bool attach_only) {
    // 1) Search existing vehicles
    auto actors = world.GetActors();
    std::string filter_lower = filter_pattern;
    // Remove "vehicle." prefix for matching
    auto pos = filter_lower.find("vehicle.");
    if (pos != std::string::npos) {
        filter_lower = filter_lower.substr(pos + 8);
    }
    // Convert to lowercase
    for (auto& c : filter_lower) c = std::tolower(c);

    carla::SharedPtr<carla::client::Actor> found_vehicle;
    for (const auto& actor : *actors) {
        std::string type_id = actor->GetTypeId();
        std::string type_lower = type_id;
        for (auto& c : type_lower) c = std::tolower(c);

        if (type_lower.find("vehicle.") != std::string::npos) {
            if (type_lower.find(filter_lower) != std::string::npos) {
                std::cout << "[OK] Mevcut araç bulundu: " << type_id
                          << "  id=" << actor->GetId() << std::endl;
                return actor;
            }
            if (!found_vehicle) {
                found_vehicle = actor;  // fallback: any vehicle
            }
        }
    }

    if (found_vehicle) {
        std::cout << "[OK] Filtre eşleşmedi, mevcut araç kullanılıyor: "
                  << found_vehicle->GetTypeId()
                  << "  id=" << found_vehicle->GetId() << std::endl;
        return found_vehicle;
    }

    if (attach_only) {
        std::cerr << "[ERROR] --attach-only seçili ama dünyada araç yok. "
                  << "Önce myvehicle_control.py'yi çalıştırın." << std::endl;
        std::exit(1);
    }

    // 2) Spawn new vehicle
    std::cout << "[INFO] Dünyada araç bulunamadı, yeni araç spawn ediliyor..." << std::endl;
    auto bp_lib = world.GetBlueprintLibrary();
    auto bps = bp_lib->Filter(filter_pattern);
    if (bps->empty()) {
        std::cerr << "[ERROR] '" << filter_pattern
                  << "' filtresiyle blueprint bulunamadı." << std::endl;
        auto all_vehicles = bp_lib->Filter("vehicle.*");
        for (auto& v : *all_vehicles) {
            std::cerr << "  " << v.GetId() << std::endl;
        }
        std::exit(1);
    }

    auto bp = (*bps)[0];
    if (bp.ContainsAttribute("role_name")) {
        bp.SetAttribute("role_name", "hero");
    }

    auto spawn_points = world.GetMap()->GetRecommendedSpawnPoints();
    if (spawn_points.empty()) {
        std::cerr << "[ERROR] Haritada spawn noktası yok." << std::endl;
        std::exit(1);
    }

    carla::SharedPtr<carla::client::Actor> vehicle;
    for (auto& sp : spawn_points) {
        sp.location.z += 0.5f;
        vehicle = world.TrySpawnActor(bp, sp);
        if (vehicle) break;
    }

    if (!vehicle) {
        std::cerr << "[ERROR] Araç spawn edilemedi." << std::endl;
        std::exit(1);
    }

    std::cout << "[OK] Araç spawn edildi: " << vehicle->GetTypeId()
              << "  id=" << vehicle->GetId() << std::endl;
    return vehicle;
}

// ── main ──────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    // ── ROS 2 init ──────────────────────────────────────────────────────
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("carla_sensor_publisher");

    // Declare parameters
    node->declare_parameter<std::string>("host", "127.0.0.1");
    node->declare_parameter<int>("port", 2000);
    node->declare_parameter<double>("timeout", 10.0);
    node->declare_parameter<std::string>("filter", "vehicle.MyVehicle");
    node->declare_parameter<bool>("sync", false);
    node->declare_parameter<bool>("attach_only", false);

    std::string host = node->get_parameter("host").as_string();
    int port = node->get_parameter("port").as_int();
    double timeout = node->get_parameter("timeout").as_double();
    std::string filter = node->get_parameter("filter").as_string();
    bool sync_mode = node->get_parameter("sync").as_bool();
    bool attach_only = node->get_parameter("attach_only").as_bool();

    RCLCPP_INFO(node->get_logger(), "[ROS2] Node 'carla_sensor_publisher' başlatıldı.");

    // ── CARLA connection ────────────────────────────────────────────────
    carla::client::Client client(host, static_cast<uint16_t>(port));
    client.SetTimeout(carla::time_duration::seconds(static_cast<size_t>(timeout)));

    carla::client::World world = client.GetWorld();
    RCLCPP_INFO(node->get_logger(), "[CARLA] Bağlandı: %s",
                world.GetMap()->GetName().c_str());

    // ── CARLA settings ──────────────────────────────────────────────────
    auto settings = world.GetSettings();
    settings.no_rendering_mode = false;

    if (sync_mode) {
        settings.synchronous_mode = true;
        settings.fixed_delta_seconds = 0.05;
        RCLCPP_INFO(node->get_logger(), "[CARLA] Senkron mod: 20 Hz sabit adım");
    } else {
        settings.synchronous_mode = false;
        settings.fixed_delta_seconds = 0.05; // Ya da asenkron ise ms_duration'a göre ayarla
        RCLCPP_INFO(node->get_logger(),
                    "[CARLA] Asenkron mod: gerçek zamanlı (fixed_delta_seconds=0)");
    }
    world.ApplySettings(settings, carla::time_duration::milliseconds(1000));

    // ── Vehicle & sensors ───────────────────────────────────────────────
    auto vehicle = findOrSpawnVehicle(world, filter, attach_only);

    auto sensor_mgr = std::make_unique<SensorManager>(world, vehicle, node);
    sensor_mgr->broadcastStaticTFs(node);

    std::unique_ptr<VehicleController> vehicle_ctrl;
    vehicle_ctrl = std::make_unique<VehicleController>(vehicle, node);

    // ── ROS 2 executor (multi-threaded) ─────────────────────────────────
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 4);
    executor->add_node(node);

    std::thread spin_thread([executor]() {
        try {
            executor->spin();
        } catch (...) {}
    });

    // ── Signal handler ──────────────────────────────────────────────────
    std::signal(SIGINT, signalHandler);

    // ── Print topic table ───────────────────────────────────────────────
    std::cout << "\n[RUNNING] Ctrl+C ile durdurun.\n" << std::endl;
    std::cout << std::left << std::setw(45) << "Topic" << "Sensor" << std::endl;
    std::cout << std::string(65, '-') << std::endl;
    std::cout << std::left << std::setw(45) << "/ouster/points"
              << "Ouster OS0-64 LiDAR" << std::endl;
    std::cout << std::left << std::setw(45) << "/velodyne/points"
              << "Velodyne LiDAR" << std::endl;
    std::cout << std::left << std::setw(45) << "/zed/zed_node/left/image_rect_color"
              << "ZED 2 Kamera Sol" << std::endl;
    std::cout << std::left << std::setw(45) << "/zed/left/camera_info"
              << "ZED 2 CameraInfo Sol" << std::endl;
    std::cout << std::left << std::setw(45) << "/zed/right/image_raw"
              << "ZED 2 Kamera Sağ" << std::endl;
    std::cout << std::left << std::setw(45) << "/zed/right/camera_info"
              << "ZED 2 CameraInfo Sağ" << std::endl;
    std::cout << std::left << std::setw(45) << "/sbg/ros/imu/data"
              << "IMU 1/SBG (xyz: 0.40,-0.12,0.80)" << std::endl;
    std::cout << std::left << std::setw(45) << "/imu/imu2/data"
              << "IMU 2 (xyz: 0.40, 0.00,0.80)" << std::endl;
    std::cout << std::left << std::setw(45) << "/sbg/ros/nav_sat_fix"
              << "GNSS/SBG NavSatFix (xyz: 1.35,-0.45,0.30)" << std::endl;
    std::cout << std::left << std::setw(45) << "/gnss/rear_right/fix"
              << "GNSS Arka Sağ (xyz: 0.00,-0.45,0.30)" << std::endl;
    std::cout << std::string(65, '-') << std::endl;

    // ── Main loop ───────────────────────────────────────────────────────
    while (!g_stop) {
        if (sync_mode) {
            world.Tick(carla::time_duration::seconds(10));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // ── Cleanup ─────────────────────────────────────────────────────────
    std::cout << "[INFO] Sensörler ve kontrol yok ediliyor..." << std::endl;

    if (vehicle_ctrl) vehicle_ctrl->destroy();
    sensor_mgr->destroy();

    // Exit sync mode
    if (sync_mode) {
        settings.synchronous_mode = false;
        world.ApplySettings(settings, carla::time_duration::milliseconds(1000));
    }

    executor->cancel();
    if (spin_thread.joinable()) spin_thread.join();

    rclcpp::shutdown();
    std::cout << "[INFO] Temizlik tamamlandı. Çıkılıyor." << std::endl;

    return 0;
}
