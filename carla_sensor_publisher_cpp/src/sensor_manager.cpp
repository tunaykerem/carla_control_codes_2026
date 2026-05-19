// Copyright (c) 2025. MIT License.
// sensor_manager.cpp — Manages all sensors + static TF broadcaster

#include "carla_sensor_publisher/sensor_manager.hpp"
#include "carla_sensor_publisher/transforms.hpp"

#include <cmath>
#include <iostream>

namespace carla_sensor_publisher {

SensorManager::SensorManager(carla::client::World& world,
                             carla::SharedPtr<carla::client::Actor> vehicle,
                             rclcpp::Node::SharedPtr node)
    : vehicle_(vehicle) {

    std::cout << "[SensorManager] Sensörler ekleniyor..." << std::endl;

    ouster_ = std::make_unique<OusterLidar>(vehicle, node);
    std::cout << "  ✓ Ouster OS0-64 LiDAR" << std::endl;

    velodyne_ = std::make_unique<VelodyneLidar>(vehicle, node);
    std::cout << "  ✓ Velodyne LiDAR" << std::endl;

    zed_ = std::make_unique<ZedCamera>(vehicle, node);
    std::cout << "  ✓ ZED 2 Stereo Kamera (sol + sağ)" << std::endl;

    imu_ = std::make_unique<IMUPublisher>(vehicle, node);
    std::cout << "  ✓ IMU 1 & 2" << std::endl;

    gnss_ = std::make_unique<GNSSPublisher>(vehicle, node);
    std::cout << "  ✓ GNSS (ön sağ + arka sağ)" << std::endl;

    std::cout << "[SensorManager] Tüm sensörler hazır." << std::endl;
}

SensorManager::~SensorManager() {
    destroy();
}

void SensorManager::destroy() {
    if (ouster_)   ouster_->destroy();
    if (velodyne_) velodyne_->destroy();
    if (zed_)      zed_->destroy();
    if (imu_)      imu_->destroy();
    if (gnss_)     gnss_->destroy();

    if (vehicle_) {
        try { vehicle_->Destroy(); } catch (...) {}
        vehicle_ = nullptr;
    }
}

void SensorManager::broadcastStaticTFs(rclcpp::Node::SharedPtr node) {
    if (!node) return;

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    const std::map<std::string, std::string> frame_map = {
        {"velodyne",         "velodyne"},
        {"ouster",           "ouster"},
        {"zed_left",         "zed_left_camera_optical_frame"},
        {"zed_right",        "zed_right_camera_optical_frame"},
        {"imu_1",            "imu_link"},
        {"imu_2",            "imu_link_2"},
        {"gnss_front_right", "imu_link"},      // real vehicle: SBG GNSS uses imu_link frame
        {"gnss_rear_right",  "gnss_rear_right"},
    };

    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for (const auto& [key, child_frame] : frame_map) {
        const auto& pose = getTransforms().at(key);
        double x     = pose[0];
        double y     = pose[1];
        double z     = pose[2];
        double roll  = pose[3];
        double pitch = pose[4];
        double yaw   = pose[5];

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = node->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = child_frame;
        t.transform.translation.x =  x;
        t.transform.translation.y = -y;   // CARLA Y=right → ROS Y=left
        t.transform.translation.z =  z;

        // Euler → Quaternion
        double cr = std::cos(roll  / 2.0), sr = std::sin(roll  / 2.0);
        double cp = std::cos(pitch / 2.0), sp = std::sin(pitch / 2.0);
        double cy = std::cos(yaw   / 2.0), sy = std::sin(yaw   / 2.0);
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy;
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy;
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy;
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy;

        tfs.push_back(t);
    }

    tf_broadcaster_->sendTransform(tfs);
    RCLCPP_INFO(node->get_logger(),
                "[TF] %zu adet statik TF yayınlandı (base_link → sensörler)",
                tfs.size());
}

}  // namespace carla_sensor_publisher
