#!/usr/bin/env python3

# Copyright (c) 2025. MIT License.
"""
MyVehicle Sensor Publisher - CARLA 0.10 / UE5
=============================================

Araç üzerindeki sensörlerin tamamını doğru TF konumlarıyla CARLA'ya ekler
ve ROS 2 topic'lerine yayınlar.

Sensörler (URDF'den alınan TF'ler):
  - Ouster OS0-64 LiDAR       : /ouster/points             (xyz: 0.85, 0.0, 1.10 | rpy: 0,0,π)
  - Velodyne LiDAR             : /velodyne/points           (xyz: 0.8, 0.0, 1.25  | rpy: 0,0,0)
  - ZED Camera (sol/sag)       : /zed/left/image_raw,
                                 /zed/right/image_raw       (Ouster'a göre ofset)
  - ZED Camera Info            : /zed/left/camera_info,
                                 /zed/right/camera_info
  - IMU 1                      : /imu/imu1/data             (xyz: 0.40,-0.12,0.80)
  - IMU 2                      : /imu/imu2/data             (xyz: 0.40, 0.00,0.80)
  - GNSS Ön Sağ                : /gnss/front_right/fix      (xyz: 1.35,-0.45,0.30)
  - GNSS Arka Sağ              : /gnss/rear_right/fix       (xyz: 0.00,-0.45,0.30)

Kullanım:
  python3 myvehicle_sensors.py [--host HOST] [--port PORT] [--filter FILTER]
                               [--sync] [--ros]

  --ros  : ROS 2 'rclpy' ile topic yayını etkinleştir (rclpy kurulu olmalı)
           kapalıysa sadece konsola veri basar.
"""

import carla
import argparse
import math
import sys
import time
import weakref
import threading
import random
import signal
from concurrent.futures import ThreadPoolExecutor

# Global thread pool – LiDAR/IMU/GNSS callback'leri buradan çalışır
_POOL = ThreadPoolExecutor(max_workers=4, thread_name_prefix='sensor')

# Kameralara ayrılmış thread pool – LiDAR callback'lerinin kamerayı bloklamasını önler
# Sol + sağ kamera aynı anda yayınlayabilsin diye en az 2 worker gerekli; 4 bıraktık
_CAM_POOL = ThreadPoolExecutor(max_workers=4, thread_name_prefix='cam')

# ── opsiyonel ROS 2 ──────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField, Imu, NavSatFix, NavSatStatus
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import Header
    import tf2_ros
    import numpy as np
    ROS_AVAILABLE = True
    try:
        from gae_msgs.msg import GaeControlCmd
        GAE_AVAILABLE = True
    except ImportError:
        GAE_AVAILABLE = False
        print("[WARNING] gae_msgs bulunamadı. Araç kontrol aboneliği devre dışı.")
except ImportError:
    ROS_AVAILABLE = False
    GAE_AVAILABLE = False
    print("[WARNING] rclpy bulunamadı. --ros bayrağı devre dışı.")

try:
    from cv_bridge import CvBridge
    BRIDGE = CvBridge()
except ImportError:
    BRIDGE = None
    print("[WARNING] cv_bridge bulunamadı. Standart yöntem kullanılacak (yavaş olabilir).")

# ── blueprint filtresi ────────────────────────────────────────────────────────
MY_VEHICLE_FILTER = 'vehicle.MyVehicle'

# ─────────────────────────────────────────────────────────────────────────────
# URDF TF Tanımları
# ─────────────────────────────────────────────────────────────────────────────
# CARLA koordinat sistemi: X=ileri, Y=SAĞ, Z=yukarı  (SOL-EL koordinat sistemi)
# ROS/URDF koordinat sistemi: X=ileri, Y=SOL, Z=yukarı  (SAĞ-EL koordinat sistemi)
# DİKKAT: Sensör verilerinde Y ekseni callback'lerde ters çevrilir.

TRANSFORMS = {
    # [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]  – CARLA koordinat sistemi
    # ÖNEMLİ: ROS/URDF'de Y=sol, CARLA'da Y=SAĞ olduğundan tüm Y değerleri ters çevrildi.
    # URDF'den CARLA'ya dönüşüm: carla_y = -urdf_y

    # LiDAR – 360° yatay FOV olduğu için yaw önemsiz, 0 bırakıldı
    'velodyne':          (0.80,  0.00, 1.25,  0.0,  0.0, 0.0),

    # Ouster: URDF yaw=π ama 360° LiDAR → yaw değişmez, aynı kalır
    'ouster':            (0.85,  0.00, 1.10,  0.0,  0.0, 0.0),

    # ZED 2 – ileri bakan (yaw=0)
    # URDF y=0 (center), sağ kamera URDF y=+0.12 = CARLA y=-0.12 (sol yönde değil, ZED için baseline negatif)
    # Gerçek uygulamada: sol lens sol tarafta (CARLA y=+0.06), sağ lens sağ (CARLA y=-0.06)
    # Ama orijinal URDF sadece sol optik frame tanımladığı için merkeze koyuyoruz
    'zed_left':          (0.85, -0.06, 1.00,  0.0,  0.0, 0.0),   # CARLA Y=-0.06 → sol taraf
    'zed_right':         (0.85,  0.06, 1.00,  0.0,  0.0, 0.0),   # CARLA Y=+0.06 → sağ taraf

    # IMU – URDF y=-0.12 → CARLA y=+0.12
    'imu_1':             (0.40,  0.12, 0.80,  0.0,  0.0, 0.0),
    'imu_2':             (0.40,  0.00, 0.80,  0.0,  0.0, 0.0),

    # GNSS – URDF y=-0.45 (sağ taraf ROS) → CARLA y=+0.45 (sağ taraf CARLA)
    'gnss_front_right':  (1.35,  0.45, 0.30,  0.0,  0.0, 0.0),
    'gnss_rear_right':   (0.00,  0.45, 0.30,  0.0,  0.0, 0.0),
}

def urdf_to_carla_transform(xyzrpy, yaw_extra_deg=0.0):
    """URDF x,y,z,roll,pitch,yaw → carla.Transform"""
    x, y, z, roll, pitch, yaw = xyzrpy
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(
            roll=math.degrees(roll),
            pitch=math.degrees(pitch),
            yaw=math.degrees(yaw) + yaw_extra_deg
        )
    )

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 Yardımcıları
# ─────────────────────────────────────────────────────────────────────────────

def make_header(frame_id: str, stamp=None):
    """rclpy Time → std_msgs/Header"""
    h = Header()
    h.frame_id = frame_id
    if stamp is not None:
        h.stamp = stamp
    return h


def _carla_stamp(data):
    """
    CARLA sensor verisinden ROS2 timestamp üret.
    data.timestamp: simülasyon saniyesi (float).
    Bu yöntem wallclock/simclock uyumsuzluğundan kaynaklanan
    görünür gecikmeyi ortadan kaldırır.
    """
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec     = int(data.timestamp)
    t.nanosec = int((data.timestamp - t.sec) * 1e9)
    return t

# ─────────────────────────────────────────────────────────────────────────────
# Ouster OS0-64 LiDAR Sensörü
# ─────────────────────────────────────────────────────────────────────────────

class OusterLidar:
    """
    Ouster OS0-64 – hızlı mod
    Topic: /ouster/points  (sensor_msgs/PointCloud2)
    """
    TOPIC = '/ouster/points'
    FRAME_ID = 'ouster'
    MIN_RANGE_SQ = 0.1  # 0.1m² (mesafe karesi olarak)

    # PointField'lar sabit – her callback'te yeniden oluşturulmasın
    _FIELDS = None

    def __init__(self, parent_actor, ros_node=None):
        self._parent = parent_actor
        self._node = ros_node
        self.sensor = None
        self._pub = None

        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('channels',          '64')      # 64→32 hız için
        bp.set_attribute('range',             '50')
        bp.set_attribute('points_per_second', '655360')   # minimal
        bp.set_attribute('rotation_frequency','20')
        bp.set_attribute('upper_fov',         '22.5')
        bp.set_attribute('lower_fov',         '-45.0')
        bp.set_attribute('horizontal_fov',    '360')
        bp.set_attribute('atmosphere_attenuation_rate', '0.0')
        bp.set_attribute('sensor_tick',       '0.05')   # 20 Hz callback

        tf = urdf_to_carla_transform(TRANSFORMS['ouster'])
        self.sensor = world.spawn_actor(
            bp, tf, attach_to=parent_actor,
            attachment_type=carla.AttachmentType.Rigid)
        try:
            self.sensor.disable_for_ros()
        except Exception:
            pass

        if ros_node is not None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5)
            self._pub = ros_node.create_publisher(PointCloud2, self.TOPIC, qos)

            if OusterLidar._FIELDS is None:
                OusterLidar._FIELDS = [
                    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]

        weak = weakref.ref(self)
        self.sensor.listen(lambda data: OusterLidar._callback(weak, data))

    @staticmethod
    def _callback(weak, data):
        self = weak()
        if not self or self._pub is None:
            return

        import numpy as np
        pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
        # 1m filtre – vektörize, sqrt yok
        mask = pts[:, 0]**2 + pts[:, 1]**2 + pts[:, 2]**2 >= OusterLidar.MIN_RANGE_SQ
        pts = pts[mask]
        # CARLA→ROS: Y eksenini ters çevir (CARLA Y=sağ → ROS Y=sol)
        pts[:, 1] *= -1.0

        msg = PointCloud2()
        msg.header = make_header(OusterLidar.FRAME_ID, _carla_stamp(data))
        msg.height = 1
        msg.width  = len(pts)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step   = msg.point_step * msg.width
        msg.fields = OusterLidar._FIELDS
        msg.data = pts.tobytes()
        self._pub.publish(msg)

    def destroy(self):
        if self.sensor and self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# Velodyne LiDAR – hızlı mod
# ─────────────────────────────────────────────────────────────────────────────

class VelodyneLidar:
    """
    Velodyne – hızlı mod, zero-copy
    Topic: /velodyne/points  (sensor_msgs/PointCloud2)
    """
    TOPIC = '/velodyne/points'
    FRAME_ID = 'velodyne'
    _FIELDS = None

    def __init__(self, parent_actor, ros_node=None):
        self._parent = parent_actor
        self._node = ros_node
        self.sensor = None
        self._pub = None

        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('channels',          '16')       # 64→32 hız için
        bp.set_attribute('range',             '100')
        bp.set_attribute('points_per_second', '300000')   # 1.3M→131k
        bp.set_attribute('rotation_frequency','10')
        bp.set_attribute('upper_fov',         '15.5')
        bp.set_attribute('lower_fov',         '-45.0')
        bp.set_attribute('horizontal_fov',    '360')
        bp.set_attribute('atmosphere_attenuation_rate', '0.0')
        bp.set_attribute('sensor_tick',       '0.1')    # 10 Hz callback

        tf = urdf_to_carla_transform(TRANSFORMS['velodyne'])
        self.sensor = world.spawn_actor(
            bp, tf, attach_to=parent_actor,
            attachment_type=carla.AttachmentType.Rigid)
        try:
            self.sensor.disable_for_ros()
        except Exception:
            pass

        if ros_node is not None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5)
            self._pub = ros_node.create_publisher(PointCloud2, self.TOPIC, qos)

            if VelodyneLidar._FIELDS is None:
                VelodyneLidar._FIELDS = [
                    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]

        weak = weakref.ref(self)
        self.sensor.listen(lambda data: VelodyneLidar._callback(weak, data))

    @staticmethod
    def _callback(weak, data):
        self = weak()
        if not self or self._pub is None:
            return

        # CARLA→ROS: Y eksenini ters çevir (CARLA Y=sağ → ROS Y=sol)
        import numpy as np
        pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        pts[:, 1] *= -1.0
        n_points = len(pts)
        raw = pts.tobytes()

        msg = PointCloud2()
        msg.header = make_header(VelodyneLidar.FRAME_ID, _carla_stamp(data))
        msg.height = 1
        msg.width  = n_points
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step   = 16 * n_points
        msg.fields = VelodyneLidar._FIELDS
        msg.data = raw
        self._pub.publish(msg)

    def destroy(self):
        if self.sensor and self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# ZED 2 Kamera – yüksek Hz modu
# ─────────────────────────────────────────────────────────────────────────────

class ZedCamera:
    """
    ZED 2 stereo kamera – Fixed high-Hz version.
    """
    IMAGE_W = 640
    IMAGE_H = 360
    FOV     = 90.0
    GAMMA   = 2.2
    BRIGHTNESS_SCALE = 3.5

    def __init__(self, parent_actor, ros_node=None):
        self._parent = parent_actor
        self._node = ros_node
        self.sensor_left  = None
        self.sensor_right = None
        self._pub_left_img  = None
        self._pub_left_info = None
        self._pub_right_img  = None
        self._pub_right_info = None
        self._busy_left  = threading.Lock()
        self._busy_right = threading.Lock()
        self._ci_left  = None
        self._ci_right = None

        # Optimization: Try to use cv_bridge for 2ms build times. 
        # If not found, it falls back to a 'slightly better' manual method.
        try:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
        except ImportError:
            self.bridge = None

        world = parent_actor.get_world()
        bp_lib = world.get_blueprint_library()

        def make_camera_bp(tick_offset):
            bp = bp_lib.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', str(self.IMAGE_W))
            bp.set_attribute('image_size_y', str(self.IMAGE_H))
            bp.set_attribute('fov',          str(self.FOV))
            bp.set_attribute('sensor_tick',  tick_offset)  # 20 Hz callback
            bp.set_attribute('enable_postprocess_effects', 'False')
            return bp

        # Her iki kamera da 20 Hz; sağ kamera sol kameradan 10ms gecikmeli spawn
        # (GPU render frame'lerini stagger etmek için)
        self.sensor_left  = world.spawn_actor(make_camera_bp('0.05'), 
                                               urdf_to_carla_transform(TRANSFORMS['zed_left']),
                                               attach_to=parent_actor)
        self.sensor_right = world.spawn_actor(make_camera_bp('0.06'), 
                                               urdf_to_carla_transform(TRANSFORMS['zed_right']),
                                               attach_to=parent_actor)

        if ros_node is not None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
            self._pub_left_img   = ros_node.create_publisher(Image,      '/zed/left/image_raw',   qos)
            self._pub_left_info  = ros_node.create_publisher(CameraInfo, '/zed/left/camera_info', qos)
            self._pub_right_img  = ros_node.create_publisher(Image,      '/zed/right/image_raw',  qos)
            self._pub_right_info = ros_node.create_publisher(CameraInfo, '/zed/right/camera_info', qos)

            self._ci_left  = ZedCamera._build_camera_info(self, 'zed_left_camera_optical_frame', 'left')
            self._ci_right = ZedCamera._build_camera_info(self, 'zed_right_camera_optical_frame', 'right')

        weak = weakref.ref(self)
        self.sensor_left.listen( lambda img: ZedCamera._dispatch(weak, img, 'left'))
        self.sensor_right.listen(lambda img: ZedCamera._dispatch(weak, img, 'right'))

    @staticmethod
    def _build_camera_info(self_ref, frame_id, side='left'):
        fx = self_ref.IMAGE_W / (2.0 * math.tan(math.radians(self_ref.FOV / 2.0)))
        fy, cx, cy = fx, self_ref.IMAGE_W / 2.0, self_ref.IMAGE_H / 2.0
        tx = -fx * 0.12 if side == 'right' else 0.0
        ci = CameraInfo()
        ci.width, ci.height = self_ref.IMAGE_W, self_ref.IMAGE_H
        ci.distortion_model = 'plumb_bob'
        ci.d, ci.k, ci.r = [0.0]*5, [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.header.frame_id = frame_id
        return ci

    @staticmethod
    def _dispatch(weak, img_data, side):
        self = weak()
        if not self: return
        lock = self._busy_left if side == 'left' else self._busy_right
        
        if not lock.acquire(blocking=False):
            return # Busy processing previous frame? Drop this one.

        # Optimization: Pass raw_data memoryview directly to avoid the 'bytes()' copy
        _CAM_POOL.submit(ZedCamera._publish, self, img_data.raw_data, _carla_stamp(img_data), img_data.width, img_data.height, side, lock)

    @staticmethod
    def _publish(self, raw_data, stamp, w, h, side, lock):
        try:
            import numpy as np
            # Zero-copy view of the buffer
            array = np.frombuffer(raw_data, dtype=np.uint8).reshape((h, w, 4))
            
            # Optimized brightness (Single pass)
            # We work on a copy here to avoid messing with CARLA's internal buffer
            out = array.copy()
            out[:, :, :3] = np.clip(out[:, :, :3].astype(np.uint16) * 3, 0, 255).astype(np.uint8)

            if self.bridge:
                # FAST PATH: Uses C++ to build the message (~1-2ms)
                img_msg = self.bridge.cv2_to_imgmsg(out, encoding="bgra8")
            else:
                # SLOW PATH: Manual (but still slightly faster than your previous setup)
                img_msg = Image()
                img_msg.height, img_msg.width = h, w
                img_msg.encoding = 'bgra8'
                img_msg.step = 4 * w
                img_msg.data = out.tobytes() 

            img_msg.header = make_header(f'zed_{side}_camera_optical_frame', stamp)
            ci = self._ci_left if side == 'left' else self._ci_right
            ci.header.stamp = stamp

            if side == 'left':
                self._pub_left_img.publish(img_msg)
                self._pub_left_info.publish(ci)
            else:
                self._pub_right_img.publish(img_msg)
                self._pub_right_info.publish(ci)
        except Exception as e:
            print(f"Camera error ({side}): {e}")
        finally:
            lock.release()

    def destroy(self):
        for s in (self.sensor_left, self.sensor_right):
            if s and s.is_alive:
                s.stop()
                s.destroy()

# ─────────────────────────────────────────────────────────────────────────────
# IMU Sensörü
# ─────────────────────────────────────────────────────────────────────────────

class IMUSensorPublisher:
    """
    İki adet IMU sensörü.
    Topics:
      /imu/imu1/data   (sensor_msgs/Imu)
      /imu/imu2/data   (sensor_msgs/Imu)
    """
    def __init__(self, parent_actor, ros_node=None):
        self._parent = parent_actor
        self._node = ros_node
        self.sensor1 = None
        self.sensor2 = None
        self._pub1 = None
        self._pub2 = None

        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        bp.set_attribute('sensor_tick', '0.005')  # 200 Hz callback

        tf1 = urdf_to_carla_transform(TRANSFORMS['imu_1'])
        tf2 = urdf_to_carla_transform(TRANSFORMS['imu_2'])

        self.sensor1 = world.spawn_actor(bp, tf1, attach_to=parent_actor,
                                          attachment_type=carla.AttachmentType.Rigid)
        self.sensor2 = world.spawn_actor(bp, tf2, attach_to=parent_actor,
                                          attachment_type=carla.AttachmentType.Rigid)
        try:
            self.sensor1.disable_for_ros()
            self.sensor2.disable_for_ros()
        except Exception:
            pass

        if ros_node is not None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1)
            self._pub1 = ros_node.create_publisher(Imu, '/imu/imu1/data', qos)
            self._pub2 = ros_node.create_publisher(Imu, '/imu/imu2/data', qos)

        # HUD için ilave: imu1 değerleri saklanır
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope     = (0.0, 0.0, 0.0)
        self.compass       = 0.0

        weak = weakref.ref(self)
        self.sensor1.listen(lambda d: IMUSensorPublisher._cb(weak, d, '/imu/imu1/data', 'imu_link_1', 1))
        self.sensor2.listen(lambda d: IMUSensorPublisher._cb(weak, d, '/imu/imu2/data', 'imu_link_2', 2))

    @staticmethod
    def _cb(weak, data, topic, frame_id, idx):
        self = weak()
        if not self:
            return

        limits = (-99.9, 99.9)
        acc = (
            max(limits[0], min(limits[1], data.accelerometer.x)),
            max(limits[0], min(limits[1], data.accelerometer.y)),
            max(limits[0], min(limits[1], data.accelerometer.z)),
        )
        gyr = (
            max(limits[0], min(limits[1], data.gyroscope.x)),
            max(limits[0], min(limits[1], data.gyroscope.y)),
            max(limits[0], min(limits[1], data.gyroscope.z)),
        )

        if idx == 1:
            self.accelerometer = acc
            self.gyroscope     = gyr
            self.compass       = math.degrees(data.compass)

        if self._node is None:
            print(f"[{topic}] acc={acc}  gyr={gyr}")
            return

        stamp = _carla_stamp(data)
        msg = Imu()
        msg.header = make_header(frame_id, stamp)
        # CARLA→ROS koordinat dönüşümü (CARLA: X=ileri,Y=SAĞ,Z=yukarı → ROS: X=ileri,Y=SOL,Z=yukarı)
        # Polar vektörler (ivme): (x, -y, z)
        # Aksiyel/pseudo vektörler (açısal hız): (-x, y, -z)
        msg.linear_acceleration.x  =  acc[0]
        msg.linear_acceleration.y  = -acc[1]   # Y ters çevir
        msg.linear_acceleration.z  =  acc[2]
        msg.angular_velocity.x     = -gyr[0]   # Pseudo-vektör: x ters
        msg.angular_velocity.y     =  gyr[1]   # Pseudo-vektör: y aynı kalır
        msg.angular_velocity.z     = -gyr[2]   # Pseudo-vektör: z ters (sola dönüş → pozitif)
        # Kovaryans bilinmiyor → -1 ile işaretle
        msg.orientation_covariance[0]          = -1.0
        msg.angular_velocity_covariance[0]     = -1.0
        msg.linear_acceleration_covariance[0]  = -1.0

        pub = self._pub1 if idx == 1 else self._pub2
        pub.publish(msg)

    def destroy(self):
        for s in (self.sensor1, self.sensor2):
            if s and s.is_alive:
                s.stop()
                s.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# GNSS Sensörü
# ─────────────────────────────────────────────────────────────────────────────

class GNSSSensorPublisher:
    """
    Ön sağ ve arka sağ GNSS sensörleri.
    Topics:
      /gnss/front_right/fix   (sensor_msgs/NavSatFix)
      /gnss/rear_right/fix    (sensor_msgs/NavSatFix)
    """
    def __init__(self, parent_actor, ros_node=None):
        self._parent = parent_actor
        self._node = ros_node
        self.sensor_front = None
        self.sensor_rear  = None
        self._pub_front = None
        self._pub_rear  = None

        # HUD için
        self.lat = 0.0
        self.lon = 0.0

        world = parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        bp.set_attribute('sensor_tick', '0.1')  # 10 Hz callback

        tf_front = urdf_to_carla_transform(TRANSFORMS['gnss_front_right'])
        tf_rear  = urdf_to_carla_transform(TRANSFORMS['gnss_rear_right'])

        self.sensor_front = world.spawn_actor(bp, tf_front, attach_to=parent_actor,
                                               attachment_type=carla.AttachmentType.Rigid)
        self.sensor_rear  = world.spawn_actor(bp, tf_rear,  attach_to=parent_actor,
                                               attachment_type=carla.AttachmentType.Rigid)
        try:
            self.sensor_front.disable_for_ros()
            self.sensor_rear.disable_for_ros()
        except Exception:
            pass

        if ros_node is not None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1)
            self._pub_front = ros_node.create_publisher(NavSatFix, '/gnss/front_right/fix', qos)
            self._pub_rear  = ros_node.create_publisher(NavSatFix, '/gnss/rear_right/fix',  qos)

        weak = weakref.ref(self)
        self.sensor_front.listen(lambda d: GNSSSensorPublisher._cb(weak, d, 'gnss_front_right', 'front'))
        self.sensor_rear.listen( lambda d: GNSSSensorPublisher._cb(weak, d, 'gnss_rear_right',  'rear'))

    @staticmethod
    def _cb(weak, data, frame_id, side):
        self = weak()
        if not self:
            return

        if side == 'front':
            self.lat = data.latitude
            self.lon = data.longitude

        if self._node is None:
            print(f"[GNSS/{side}] lat={data.latitude:.6f}  lon={data.longitude:.6f}  alt={data.altitude:.2f}")
            return

        stamp = _carla_stamp(data)
        msg = NavSatFix()
        msg.header              = make_header(frame_id, stamp)
        msg.status.status       = NavSatStatus.STATUS_FIX
        msg.status.service      = NavSatStatus.SERVICE_GPS
        msg.latitude            = data.latitude
        msg.longitude           = data.longitude
        msg.altitude            = data.altitude
        msg.position_covariance = [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        pub = self._pub_front if side == 'front' else self._pub_rear
        pub.publish(msg)

    def destroy(self):
        for s in (self.sensor_front, self.sensor_rear):
            if s and s.is_alive:
                s.stop()
                s.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# TF Broadcaster (statik)
# ─────────────────────────────────────────────────────────────────────────────

def broadcast_static_tfs(node):
    """
    base_link → her sensör çerçevesine statik TF yayını.
    """
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    import math

    broadcaster = StaticTransformBroadcaster(node)
    tfs = []

    frame_map = {
        'velodyne':          'velodyne',
        'ouster':            'ouster',
        'zed_left':          'zed_left_camera_optical_frame',
        'zed_right':         'zed_right_camera_optical_frame',
        'imu_1':             'imu_link_1',
        'imu_2':             'imu_link_2',
        'gnss_front_right':  'gnss_front_right',
        'gnss_rear_right':   'gnss_rear_right',
    }

    for key, child_frame in frame_map.items():
        x, y, z, roll, pitch, yaw = TRANSFORMS[key]
        t = TransformStamped()
        t.header.stamp          = node.get_clock().now().to_msg()
        t.header.frame_id       = 'base_link'
        t.child_frame_id        = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = -y   # CARLA Y=sağ → ROS Y=sol
        t.transform.translation.z = z

        # Euler → Quaternion
        cr = math.cos(roll  / 2); sr = math.sin(roll  / 2)
        cp = math.cos(pitch / 2); sp = math.sin(pitch / 2)
        cy = math.cos(yaw   / 2); sy = math.sin(yaw   / 2)
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy

        tfs.append(t)

    broadcaster.sendTransform(tfs)
    node.get_logger().info(f"[TF] {len(tfs)} adet statik TF yayınlandı (base_link → sensörler)")
    return broadcaster   # referansı koruyun


# ─────────────────────────────────────────────────────────────────────────────
# Araç Kontrol (GaeControlCmd → CARLA VehicleControl + PID hız kontrolü)
# ─────────────────────────────────────────────────────────────────────────────

class PIDController:
    """Basit PID kontrolcü."""
    def __init__(self, kp=0.5, ki=0.05, kd=0.1, out_min=0.0, out_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def step(self, error, dt):
        if dt <= 0:
            return 0.0
        self._integral += error * dt
        # Anti-windup: integral sınırla
        self._integral = max(-5.0, min(5.0, self._integral))
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, output))


class VehicleController:
    """
    /vehicle/control (GaeControlCmd) topic'ini dinler ve CARLA aracını sürer.

    GaeControlCmd alanları:
      throttle:         0-200   (RPM benzeri, 200 ≈ 35 km/h)
      steering:         0-3600  (0=tam sağ, 1800=düz, 3600=tam sol)
      brake:            0-10000
      mechanical_brake: 0-1
      gear:             0=boş, 1=ileri, 2=geri
      mode_auto:        0=manuel, 1=otonom
      signal:           0-3

    CARLA VehicleControl:
      throttle: 0.0-1.0
      steer:    -1.0 (sol) … +1.0 (sağ)  [CARLA koordinatı]
      brake:    0.0-1.0
      reverse:  bool
      hand_brake: bool
    """
    # 200 RPM ≈ 35 km/h  →  1 RPM ≈ 0.175 km/h
    RPM_TO_KMH = 35.0 / 200.0   # 0.175

    TOPIC = '/vehicle/control'

    def __init__(self, vehicle, ros_node):
        self._vehicle = vehicle
        self._node = ros_node
        self._enabled = False

        # PID hız kontrolcüsü
        self._speed_pid = PIDController(kp=0.5, ki=0.05, kd=0.1, out_min=0.0, out_max=1.0)
        self._brake_pid = PIDController(kp=0.3, ki=0.0,  kd=0.05, out_min=0.0, out_max=1.0)

        # Son gelen komut (thread-safe)
        self._lock = threading.Lock()
        self._last_cmd = None        # GaeControlCmd
        self._last_cmd_time = None   # wall-clock
        self._cmd_timeout = 1.0      # 1s içinde komut gelmezse dur

        # Subscriber
        if ros_node is not None and GAE_AVAILABLE:
            ros_node.create_subscription(
                GaeControlCmd, self.TOPIC,
                self._on_cmd, 10
            )
            # 20 Hz kontrol döngüsü (ROS timer)
            self._timer = ros_node.create_timer(0.05, self._control_loop)
            self._enabled = True
            print(f"  ✓ VehicleController: {self.TOPIC} dinleniyor (PID hız kontrolü)")
        else:
            print("  ✗ VehicleController: gae_msgs yok veya ROS kapalı, araç kontrolü devre dışı")

    def _on_cmd(self, msg):
        """GaeControlCmd geldiğinde sakla."""
        with self._lock:
            self._last_cmd = msg
            self._last_cmd_time = time.time()

    def _get_current_speed_kmh(self):
        """Aracın anlık hızını km/h olarak al."""
        v = self._vehicle.get_velocity()
        speed_ms = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        return speed_ms * 3.6

    def _control_loop(self):
        """20 Hz'de çalışan ana kontrol döngüsü."""
        if not self._enabled:
            return

        with self._lock:
            cmd = self._last_cmd
            cmd_time = self._last_cmd_time

        # Komut yok veya timeout → dur
        if cmd is None or cmd_time is None:
            return
        if time.time() - cmd_time > self._cmd_timeout:
            # Timeout: acil dur
            ctrl = carla.VehicleControl()
            ctrl.throttle = 0.0
            ctrl.brake = 1.0
            ctrl.steer = 0.0
            ctrl.hand_brake = False
            ctrl.reverse = False
            self._vehicle.apply_control(ctrl)
            return

        # ── Steering dönüştür ────────────────────────────────────────────
        # GaeControlCmd: 0=tam sağ, 1800=düz, 3600=tam sol
        # CARLA steer:  +1.0=tam sağ, 0=düz, -1.0=tam sol
        steer = (1800.0 - float(cmd.steering)) / 1800.0
        steer = max(-1.0, min(1.0, steer))

        # ── Gear / Reverse ───────────────────────────────────────────────
        reverse = (cmd.gear == 2)

        # ── Hedef hız ve PID ─────────────────────────────────────────────
        target_speed_kmh = float(cmd.throttle) * self.RPM_TO_KMH
        current_speed_kmh = self._get_current_speed_kmh()
        speed_error = target_speed_kmh - current_speed_kmh

        dt = 0.05  # 20 Hz

        # Manuel fren (GaeControlCmd.brake 0-10000)
        manual_brake = float(cmd.brake) / 10000.0

        if cmd.throttle == 0 and manual_brake < 0.01:
            # Gaz yok, fren yok → hafif fren (sürüklenmesin)
            carla_throttle = 0.0
            carla_brake = 0.05
        elif manual_brake > 0.01:
            # Manuel fren aktif
            carla_throttle = 0.0
            carla_brake = max(manual_brake, 0.1)
            self._speed_pid.reset()
        elif speed_error > 0:
            # Hedef hıza çıkmak için gaz ver
            carla_throttle = self._speed_pid.step(speed_error, dt)
            carla_brake = 0.0
        else:
            # Hedef hızın üstünde → frenle
            carla_throttle = 0.0
            carla_brake = self._brake_pid.step(-speed_error, dt)
            self._speed_pid.reset()

        # ── Hand brake ───────────────────────────────────────────────────
        hand_brake = bool(cmd.mechanical_brake)

        # ── CARLA kontrolü uygula ────────────────────────────────────────
        ctrl = carla.VehicleControl()
        ctrl.throttle   = carla_throttle
        ctrl.steer      = steer
        ctrl.brake      = carla_brake
        ctrl.hand_brake = hand_brake
        ctrl.reverse    = reverse

        self._vehicle.apply_control(ctrl)

    def destroy(self):
        """Durdur."""
        self._enabled = False
        try:
            ctrl = carla.VehicleControl()
            ctrl.throttle = 0.0
            ctrl.brake = 1.0
            ctrl.steer = 0.0
            self._vehicle.apply_control(ctrl)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Ana Varlık Yöneticisi
# ─────────────────────────────────────────────────────────────────────────────

class SensorManager:
    def __init__(self, world, vehicle, ros_node=None):
        self.vehicle = vehicle
        self._sensors = []
        node = ros_node

        print("[SensorManager] Sensörler ekleniyor...")

        self.ouster   = OusterLidar(vehicle, node)
        self._sensors.append(self.ouster)
        print("  ✓ Ouster OS0-64 LiDAR")

        self.velodyne = VelodyneLidar(vehicle, node)
        self._sensors.append(self.velodyne)
        print("  ✓ Velodyne LiDAR")

        self.zed      = ZedCamera(vehicle, node)
        self._sensors.append(self.zed)
        print("  ✓ ZED 2 Stereo Kamera (sol + sağ)")

        self.imu      = IMUSensorPublisher(vehicle, node)
        self._sensors.append(self.imu)
        print("  ✓ IMU 1 & 2")

        self.gnss     = GNSSSensorPublisher(vehicle, node)
        self._sensors.append(self.gnss)
        print("  ✓ GNSS (ön sağ + arka sağ)")

        print("[SensorManager] Tüm sensörler hazır.\n")

    def destroy(self):
        for s in self._sensors:
            try:
                s.destroy()
            except Exception:
                pass
        if self.vehicle and self.vehicle.is_alive:
            self.vehicle.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# Araç spawn
# ─────────────────────────────────────────────────────────────────────────────

def find_or_spawn_vehicle(world, filter_pattern='vehicle.MyVehicle', attach_only=False):
    """
    Önce dünyada zaten çalışan araç ara (myvehicle_control.py tarafından spawn edilmiş).
    Bulamazsa (attach_only=False ise) yeni araç spawn eder.
    """
    # 1) Mevcut araçları tara
    actors = world.get_actors().filter('vehicle.*')
    filter_lower = filter_pattern.lower().replace('vehicle.', '')
    for actor in actors:
        if filter_lower in actor.type_id.lower():
            print(f"[OK] Mevcut araç bulundu: {actor.type_id}  id={actor.id}")
            return actor

    # Filtre eşleşmezse herhangi bir araç dene
    if actors:
        actor = list(actors)[0]
        print(f"[OK] Filtre eşleşmedi, mevcut araç kullanılıyor: {actor.type_id}  id={actor.id}")
        return actor

    if attach_only:
        print("[ERROR] --attach-only seçili ama dünyada araç yok. Önce myvehicle_control.py'yi çalıştırın.")
        sys.exit(1)

    # 2) Yeni araç spawn et
    print("[INFO] Dünyada araç bulunamadı, yeni araç spawn ediliyor...")
    bp_lib = world.get_blueprint_library()
    bps = list(bp_lib.filter(filter_pattern))
    if not bps:
        print(f"[ERROR] '{filter_pattern}' filtresiyle blueprint bulunamadı.")
        for v in bp_lib.filter('vehicle.*'):
            print(f"  {v.id}")
        sys.exit(1)

    bp = bps[0]
    try:
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', 'hero')
    except Exception:
        pass  # CARLA 0.10'da bazı blueprint'ler desteklemiyor

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("[ERROR] Haritada spawn noktası yok.")
        sys.exit(1)

    random.shuffle(spawn_points)
    vehicle = None
    for sp in spawn_points:
        sp.location.z += 0.5
        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle:
            break

    if vehicle is None:
        print("[ERROR] Araç spawn edilemedi.")
        sys.exit(1)

    print(f"[OK] Araç spawn edildi: {vehicle.type_id}  id={vehicle.id}")
    return vehicle


# ─────────────────────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='MyVehicle Sensor Publisher - CARLA 0.10')
    ap.add_argument('--host',       default='127.0.0.1', help='CARLA sunucu IP')
    ap.add_argument('--port',  type=int, default=2000,   help='CARLA sunucu portu')
    ap.add_argument('--timeout', type=float, default=10.0, help='CARLA bağlantı zaman aşımı (s)')
    ap.add_argument('--filter',       default=MY_VEHICLE_FILTER, help='Araç blueprint filtresi')
    ap.add_argument('--sync',  action='store_true', help='Senkron mod etkinleştir')
    ap.add_argument('--ros',   action='store_true', help='ROS 2 topic yayını etkinleştir')
    ap.add_argument('--attach-only', action='store_true',
                    help='Yeni araç spawn etme; sadece mevcut araca sensör ekle')
    args = ap.parse_args()

    # ── ROS 2 kurulumu ─────────────────────────────────────────────────────
    ros_node = None
    if args.ros:
        if not ROS_AVAILABLE:
            print("[ERROR] rclpy yüklü değil. Lütfen ROS 2 ortamınızı source edin.")
            sys.exit(1)
        rclpy.init()
        ros_node = rclpy.create_node('carla_sensor_publisher')
        print("[ROS2] Node 'carla_sensor_publisher' başlatıldı.")
    else:
        print("[INFO] ROS 2 modu kapalı. Sadece konsola veri basar. (--ros ile etkinleştirin)")

    # ── CARLA bağlantı ─────────────────────────────────────────────────────
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    try:
        world = client.get_world()
        print(f"[CARLA] Bağlandı: {world.get_map().name}")
    except Exception as e:
        print(f"[ERROR] CARLA bağlantısı kurulamadı: {e}")
        sys.exit(1)

    # ── CARLA settings ─────────────────────────────────────────────────────
    settings = world.get_settings()
    
    # LiDAR için rendering ON olmalı
    settings.no_rendering_mode = False
    
    if args.sync:
        # Senkron modda: 20 Hz sabit sim adımı + tick() döngüsü
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        print("[CARLA] Senkron mod: 20 Hz sabit adım")
    else:
        # Asenkron modda: CARLA gerçek zamanlı (wall-clock) koşsun
        # fixed_delta_seconds = 0.0 → CARLA kendi frame süresini belirler
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.0
        print("[CARLA] Asenkron mod: gerçek zamanlı (fixed_delta_seconds=0)")
    
    world.apply_settings(settings)

    # ── Araç ve sensörler ──────────────────────────────────────────────────
    vehicle    = find_or_spawn_vehicle(world, args.filter,
                                       attach_only=getattr(args, 'attach_only', False))
    sensor_mgr = SensorManager(world, vehicle, ros_node)
    vehicle_ctrl = VehicleController(vehicle, ros_node) if ros_node else None
    tf_broadcaster = None

    if ros_node is not None:
        tf_broadcaster = broadcast_static_tfs(ros_node)

    # ── ROS 2 executor ────────────────────────────────────────────────────
    # rclpy.spin() GIL'i sürekli tutar ve CARLA kamera callback'lerini
    # ~5 Hz'e düşürür. MultiThreadedExecutor OS thread'lerini kullanır,
    # GIL time-slice'ları CARLA callback thread'lerine daha adil dağılır.
    executor = None
    spin_thread = None
    if ros_node is not None:
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(ros_node)

        def _spin_executor():
            try:
                executor.spin()
            except Exception:
                pass

        spin_thread = threading.Thread(target=_spin_executor, daemon=True)
        spin_thread.start()

    # ── Çalışma döngüsü ───────────────────────────────────────────────────
    stop_event = threading.Event()

    def _sigint(sig, frame):
        print("\n[INFO] Durduruluyor...")
        stop_event.set()

    signal.signal(signal.SIGINT, _sigint)

    print("\n[RUNNING] Ctrl+C ile durdurun.\n")
    print(f"{'Topic':<45} {'Sensor'}")
    print("-" * 65)
    print(f"{'/ouster/points':<45} Ouster OS0-64 LiDAR")
    print(f"{'/velodyne/points':<45} Velodyne LiDAR")
    print(f"{'/zed/left/image_raw':<45} ZED 2 Kamera Sol")
    print(f"{'/zed/left/camera_info':<45} ZED 2 CameraInfo Sol")
    print(f"{'/zed/right/image_raw':<45} ZED 2 Kamera Sağ")
    print(f"{'/zed/right/camera_info':<45} ZED 2 CameraInfo Sağ")
    print(f"{'/imu/imu1/data':<45} IMU 1 (xyz: 0.40,-0.12,0.80)")
    print(f"{'/imu/imu2/data':<45} IMU 2 (xyz: 0.40, 0.00,0.80)")
    print(f"{'/gnss/front_right/fix':<45} GNSS Ön Sağ (xyz: 1.35,-0.45,0.30)")
    print(f"{'/gnss/rear_right/fix':<45} GNSS Arka Sağ (xyz: 0.00,-0.45,0.30)")
    print("-" * 65)

    try:
        while not stop_event.is_set():
            if args.sync:
                world.tick()
            else:
                time.sleep(0.05)
    finally:
        print("[INFO] Sensörler ve kontrol yok ediliyor...")
        if vehicle_ctrl:
            vehicle_ctrl.destroy()
        sensor_mgr.destroy()

        # Senkron moddan çık
        if args.sync:
            settings.synchronous_mode = False
            world.apply_settings(settings)

        if executor is not None:
            executor.shutdown()
        if ros_node is not None:
            ros_node.destroy_node()
            rclpy.shutdown()

        print("[INFO] Temizlik tamamlandı. Çıkılıyor.")


if __name__ == '__main__':
    main()