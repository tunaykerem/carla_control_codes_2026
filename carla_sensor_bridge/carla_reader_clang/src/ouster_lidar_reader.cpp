#include "carla_reader_clang/ouster_lidar_reader.hpp"
#include "carla_reader_clang/tcp_sender.hpp"
#include <carla/client/BlueprintLibrary.h>
#include <iostream>

namespace carla_sensor_bridge {

OusterLidarReader::OusterLidarReader(carla::SharedPtr<carla::client::Actor> parent,
                                     carla::client::World& world) {
    auto bp_lib = world.GetBlueprintLibrary();
    auto lidar_bp = *(bp_lib->Find("sensor.lidar.ray_cast"));

    lidar_bp.SetAttribute("channels", "64");
    lidar_bp.SetAttribute("range", "120.0");
    lidar_bp.SetAttribute("points_per_second", "1310720");
    lidar_bp.SetAttribute("rotation_frequency", "20.0");
    lidar_bp.SetAttribute("upper_fov", "22.5");
    lidar_bp.SetAttribute("lower_fov", "-22.5");
    lidar_bp.SetAttribute("sensor_tick", "0.05");
    lidar_bp.SetAttribute("dropoff_general_rate", "0.0");
    lidar_bp.SetAttribute("dropoff_intensity_limit", "1.0");
    lidar_bp.SetAttribute("dropoff_zero_intensity", "0.0");

    carla::geom::Transform transform(
        carla::geom::Location(0.85f, 0.0f, 1.10f),
        carla::geom::Rotation(0.0f, 0.0f, 0.0f)
    );

    auto actor = world.SpawnActor(lidar_bp, transform, parent.get());
    // SpawnActor returns SharedPtr<Actor>; Listen/Stop live on Sensor subclass
    sensor_ = std::static_pointer_cast<carla::client::Sensor>(actor);

    if (sensor_) {
        sensor_->Listen([this](auto data) { this->onSensorData(data); });
        std::cout << "[CARLA Reader] Spawned Ouster LiDAR (ID: "
                  << sensor_->GetId() << ")\n";
    } else {
        std::cerr << "[CARLA Reader] ERROR: Failed to cast Actor to Sensor!\n";
    }
}

OusterLidarReader::~OusterLidarReader() {
    destroy();
}

void OusterLidarReader::destroy() {
    if (sensor_) {
        try {
            sensor_->Stop();
            sensor_->Destroy();
        } catch (...) {}
        sensor_ = nullptr;
    }
}

void OusterLidarReader::onSensorData(carla::SharedPtr<carla::sensor::SensorData> data) {
    auto lidar_data = std::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
    if (!lidar_data) return;

    size_t num_points = lidar_data->size();
    if (num_points == 0) return;

    double timestamp = lidar_data->GetTimestamp();
    
    // Each point in CARLA is 4 floats (x, y, z, intensity)
    size_t payload_size = num_points * 4 * sizeof(float);
    const uint8_t* raw_bytes = reinterpret_cast<const uint8_t*>(lidar_data->data());

    // Send via TCP Bridge
    TCPSender::getInstance().send(SensorType::OUSTER_LIDAR, timestamp, raw_bytes, payload_size);
}

} // namespace carla_sensor_bridge

