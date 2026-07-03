#pragma once

#include <memory>
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
};

} // namespace carla_sensor_bridge
