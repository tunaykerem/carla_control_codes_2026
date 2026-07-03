#include "carla_reader_clang/zed_camera_reader.hpp"
#include "carla_reader_clang/tcp_sender.hpp"
#include "profiling.hpp"
#include <carla/client/BlueprintLibrary.h>
#include <carla/image/ImageView.h>
#include <iostream>
#include <cstring>

namespace carla_sensor_bridge {

ZedCameraReader::ZedCameraReader(carla::SharedPtr<carla::client::Actor> parent,
                                 carla::client::World& world) {
    auto bp_lib = world.GetBlueprintLibrary();
    auto camera_bp = *(bp_lib->Find("sensor.camera.rgb"));

    camera_bp.SetAttribute("image_size_x", "1280");
    camera_bp.SetAttribute("image_size_y", "720");
    camera_bp.SetAttribute("fov", "90.0");
    camera_bp.SetAttribute("sensor_tick", "0.05");              // 20 Hz
    camera_bp.SetAttribute("enable_postprocess_effects", "true");

    // URDF transform for zed_left: (0.85, -0.06, 1.00, 0, 0, 0)
    carla::geom::Transform transform(
        carla::geom::Location(0.85f, -0.06f, 1.00f),
        carla::geom::Rotation(0.0f, 0.0f, 0.0f)
    );

    auto actor = world.SpawnActor(camera_bp, transform, parent.get());
    sensor_ = std::static_pointer_cast<carla::client::Sensor>(actor);

    if (sensor_) {
        sensor_->Listen([this](auto data) { this->onImageData(data); });
        std::cout << "[CARLA Reader] Spawned ZED Left Camera (ID: "
                  << sensor_->GetId() << ")\n";
    } else {
        std::cerr << "[CARLA Reader] ERROR: Failed to cast Camera Actor to Sensor!\n";
    }
}

ZedCameraReader::~ZedCameraReader() {
    destroy();
}

void ZedCameraReader::destroy() {
    if (sensor_) {
        try {
            sensor_->Stop();
            sensor_->Destroy();
        } catch (...) {}
        sensor_ = nullptr;
    }
}

void ZedCameraReader::onImageData(carla::SharedPtr<carla::sensor::SensorData> data) {
    PROF_READ_BEGIN("ZED");

    auto image = std::dynamic_pointer_cast<carla::sensor::data::Image>(data);
    if (!image) return;

    uint32_t width  = image->GetWidth();
    uint32_t height = image->GetHeight();
    if (width == 0 || height == 0) return;

    double timestamp = image->GetTimestamp();

    // CARLA image: BGRA, 4 bytes per pixel
    // We send: [width(4B) | height(4B) | BGRA pixel data]
    // The publisher side will handle BGRA→BGR8 conversion + brightness

    size_t pixel_data_size = static_cast<size_t>(width) * height * 4;
    size_t total_payload   = sizeof(uint32_t) * 2 + pixel_data_size;

    // Build payload: metadata prefix + raw pixels
    std::vector<uint8_t> payload(total_payload);
    std::memcpy(payload.data(),     &width,  sizeof(uint32_t));
    std::memcpy(payload.data() + 4, &height, sizeof(uint32_t));

    // Copy pixel data from CARLA image
    const auto* raw_pixels = reinterpret_cast<const uint8_t*>(image->data());
    std::memcpy(payload.data() + 8, raw_pixels, pixel_data_size);

    PROF_READ_T1();

    // Send via TCP Bridge
    TCPSender::getInstance().send(SensorType::ZED_LEFT_IMAGE, timestamp,
                                  payload.data(), static_cast<uint32_t>(total_payload));

    PROF_READ_END("ZED");
}

} // namespace carla_sensor_bridge
