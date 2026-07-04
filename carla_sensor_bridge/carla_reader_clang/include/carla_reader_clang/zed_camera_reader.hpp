#pragma once

#include <memory>
#include <vector>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/sensor/data/Image.h>

namespace carla_sensor_bridge {

class ZedCameraReader {
public:
    ZedCameraReader(carla::SharedPtr<carla::client::Actor> parent,
                    carla::client::World& world);
    ~ZedCameraReader();
    void destroy();

private:
    void onImageData(carla::SharedPtr<carla::sensor::SensorData> data);

    carla::SharedPtr<carla::client::Sensor> sensor_;

    // Pre-allocated buffers (reused across frames)
    std::vector<uint8_t> bgr_buffer_;    // BGRA→BGR conversion buffer
    std::vector<uint8_t> jpeg_buffer_;   // JPEG output buffer
    std::vector<uint8_t> send_buffer_;   // TCP send buffer

    static constexpr float BRIGHTNESS_SCALE = 3.5f;
    static constexpr int   JPEG_QUALITY     = 85;
};

} // namespace carla_sensor_bridge
