#pragma once

#include <memory>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/sensor/data/LidarMeasurement.h>

namespace carla_sensor_bridge {

class OusterLidarReader {
public:
    OusterLidarReader(carla::SharedPtr<carla::client::Actor> parent,
                      carla::client::World& world);
    ~OusterLidarReader();
    void destroy();

private:
    void onSensorData(carla::SharedPtr<carla::sensor::SensorData> data);

    carla::SharedPtr<carla::client::Sensor> sensor_;
};

} // namespace carla_sensor_bridge
