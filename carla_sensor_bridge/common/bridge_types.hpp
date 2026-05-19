#pragma once

#include <cstdint>

namespace carla_sensor_bridge {

enum class SensorType : uint8_t {
    OUSTER_LIDAR = 0,
    VELODYNE_LIDAR = 1,
    ZED_LEFT_IMAGE = 2,
    ZED_RIGHT_IMAGE = 3,
    GNSS_SBG = 4,
    GNSS_REAR = 5,
    IMU_SBG = 6,
    IMU_2 = 7,
    TRANSFORM = 8
};

#pragma pack(push, 1)
struct PacketHeader {
    uint32_t magic;
    SensorType type;
    double timestamp;
    uint32_t payload_size;
};
#pragma pack(pop)

constexpr uint32_t MAGIC_BYTES = 0xCACA0000;
constexpr int DEFAULT_TCP_PORT = 9090;

} // namespace carla_sensor_bridge
