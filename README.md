# CARLA Control Codes 2026

**İTÜ SCT GAE** — CARLA UE5 (0.10) simülasyonu üzerinde otonom araç sensör pipeline'ı ve kontrol sistemi.

## Mimari

Sistem üç katmandan oluşur:

```
┌─────────────────────────────────────────────────────────────┐
│                    CARLA UE5 Simülasyon                      │
│                    (CarlaUnreal.sh)                          │
└──────────┬───────────────────┬──────────────────┬───────────┘
           │ Python RPC        │ Python RPC        │ C++ RPC
           ▼                   ▼                   ▼
┌──────────────────┐ ┌─────────────────────┐ ┌────────────────────┐
│ myvehicle_control│ │ Python Sensors      │ │ carla_reader_clang │
│ .py              │ │ --no-ouster         │ │ (Clang + libc++)   │
│                  │ │                     │ │                    │
│ • Araç spawn     │ │ • Velodyne LiDAR    │ │ • Ouster LiDAR     │
│ • Klavye kontrol │ │ • ZED Kamera (L/R)  │ │   sensör okuma     │
│ • Pygame HUD     │ │ • IMU (SBG + IMU2)  │ │ • TCP ile gönder   │
│                  │ │ • GNSS (2x)         │ └────────┬───────────┘
│                  │ │ • TF Broadcast      │          │ TCP :9090
│                  │ │ • Vehicle Control   │          ▼
│                  │ └──────────┬──────────┘ ┌────────────────────┐
│                  │            │ ROS 2 DDS  │ ros2_publisher_gcc │
│                  │            ▼            │ (GCC + libstdc++)  │
│                  │    ┌──────────────┐     │                    │
│                  │    │  ROS 2       │◄────│ • /ouster/points   │
│                  │    │  Topics      │     └────────────────────┘
└──────────────────┘    └──────────────┘
```

### Neden İki Ayrı C++ Proses?

CARLA SDK **Clang + libc++** ile derlenir, ROS 2 ise **GCC + libstdc++** kullanır. Bu iki C++ standart kütüphanesi ABI uyumsuz olduğundan aynı binary'de birleştirilemez. Çözüm:

| Proses | Compiler | Görevi |
|--------|----------|--------|
| `carla_reader_clang` | Clang + libc++ | CARLA'dan ham sensör verisi okur |
| `ros2_publisher_gcc` | GCC + libstdc++ | Veriyi ROS 2 PointCloud2 olarak yayınlar |

Aralarındaki iletişim **TCP** (port 9090) üzerinden custom binary protokol ile sağlanır.

## Proje Yapısı

```
carla_control_codes_2026/
│
├── setup_env.sh                            # 🔧 Ortam değişkeni kurulum scripti
├── myvehicle_control.py                    # Araç spawn + klavye kontrol (Pygame)
├── myvehicle_sensors_carla_ekstra_topic_fix.py  # Python sensör publisher (--no-ouster desteği)
├── clang-libc++.cmake                      # Clang toolchain dosyası
│
├── carla_sensor_bridge/                    # ✅ Aktif: Bölünmüş C++ mimari
│   ├── common/
│   │   └── bridge_types.hpp                # Ortak TCP protokol tanımları
│   ├── carla_reader_clang/                 # CARLA sensör okuyucu (Clang)
│   │   ├── CMakeLists.txt
│   │   ├── include/carla_reader_clang/
│   │   │   ├── tcp_sender.hpp
│   │   │   └── ouster_lidar_reader.hpp
│   │   └── src/
│   │       ├── main.cpp
│   │       ├── tcp_sender.cpp
│   │       ├── ouster_lidar_reader.cpp
│   │       └── *.cpp (stub — henüz implemente edilmedi)
│   └── ros2_publisher_gcc/                 # ROS 2 publisher (GCC/ament_cmake)
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/ros2_publisher_gcc/
│       │   ├── tcp_receiver.hpp
│       │   └── ouster_lidar_publisher.hpp
│       └── src/
│           ├── main.cpp
│           ├── tcp_receiver.cpp
│           ├── ouster_lidar_publisher.cpp
│           └── *.cpp (stub)
│
└── carla_sensor_publisher_cpp/             # ⚠️ Eski monolitik C++ deneme (referans)
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/ & src/
```

## Sensör Listesi

| Sensör | ROS 2 Topic | Publisher | Durum |
|--------|-------------|----------|-------|
| Ouster OS0-64 LiDAR | `/ouster/points` | **C++ Bridge** | ✅ |
| Velodyne LiDAR | `/velodyne/points` | Python | ✅ |
| ZED 2 Kamera Sol | `/zed/zed_node/left/image_rect_color` | Python | ✅ |
| ZED 2 Kamera Sağ | `/zed/right/image_raw` | Python | ✅ |
| IMU SBG | `/sbg/ros/imu/data` | Python | ✅ |
| IMU 2 | `/imu/imu2/data` | Python | ✅ |
| GNSS SBG | `/sbg/ros/nav_sat_fix` | Python | ✅ |
| GNSS Arka | `/gnss/rear_right/fix` | Python | ✅ |
| TF (Static) | `/tf_static` | Python | ✅ |

## Gereksinimler

- **CARLA** 0.10.0 (UE5) — Linux Shipping build
- **ROS 2 Humble**
- **Python 3.10+** — carla, pygame, numpy, rclpy, cv_bridge
- **Clang 14+** — carla_reader_clang build için
- **GCC 11+** — ros2_publisher_gcc build için (ROS 2 ile gelir)
- **CARLA_ROOT** ortam değişkeni — CarlaUE5 kaynak dizinine işaret etmeli (bkz. [Ortam Kurulumu](#ortam-kurulumu))

## Ortam Kurulumu

Bu proje **makineye özel hardcoded yol içermez**. Build öncesinde `CARLA_ROOT` ortam değişkenini ayarlamanız gerekir. Üç yöntemden birini kullanabilirsiniz:

### Yöntem 1: setup_env.sh (Önerilen)

```bash
# Otomatik algılama (bilinen dizinleri tarar)
source setup_env.sh

# Veya yolu kendiniz belirtin
source setup_env.sh /path/to/CarlaUE5
```

### Yöntem 2: Manuel export

```bash
export CARLA_ROOT=/path/to/CarlaUE5

# Kalıcı yapmak için:
echo 'export CARLA_ROOT=/path/to/CarlaUE5' >> ~/.bashrc
```

### Yöntem 3: CMake'e doğrudan geçirme

```bash
cmake -DCARLA_ROOT=/path/to/CarlaUE5 ..
```

> ⚠️ `CARLA_ROOT` tanımlı değilse CMake açıklayıcı bir hata mesajı gösterir ve durur.

## Build

> **Ön koşul:** Yukarıdaki [Ortam Kurulumu](#ortam-kurulumu) adımını tamamlamış olun.

### 1. carla_reader_clang (Standalone CMake — Clang)

```bash
cd carla_sensor_bridge/carla_reader_clang
mkdir -p build && cd build
CC=clang CXX=clang++ cmake ..
make -j$(nproc)
```

### 2. ros2_publisher_gcc (Colcon — GCC)

```bash
source /opt/ros/humble/setup.bash
cd carla_sensor_bridge
colcon build --packages-select ros2_publisher_gcc
```

## Çalıştırma (5 Terminal — Hibrit Mod)

> **Başlatma sırası önemlidir!** Aşağıdaki sırayı takip edin.

### T1 — CARLA Simülasyon

```bash
cd ~/itu_sct_gae/carla/carla_build_mapler/carla_dubalar_eklendi/Carla-0.10.0-Linux-Shipping/Linux
./CarlaUnreal.sh
```

### T2 — Araç Kontrolü

```bash
cd ~/itu_sct_gae/carla/carla_control_codes_2026
python3 myvehicle_control.py
```

> ⚠️ `--sync` kullanmayın — çoklu client yapısında çakışma yaratır.

### T3 — Python Sensörler (Ouster hariç)

```bash
source /opt/ros/humble/setup.bash
cd ~/itu_sct_gae/carla/carla_control_codes_2026
python3 myvehicle_sensors_carla_ekstra_topic_fix.py --ros --attach-only --no-ouster
```

### T4 — ROS 2 Publisher (C++)

```bash
source /opt/ros/humble/setup.bash
source ~/itu_sct_gae/carla/carla_control_codes_2026/carla_sensor_bridge/install/setup.bash
ros2 run ros2_publisher_gcc ros2_publisher_node
```

### T5 — CARLA Reader (C++)

```bash
cd ~/itu_sct_gae/carla/carla_control_codes_2026/carla_sensor_bridge/carla_reader_clang/build
./carla_reader_node
```

### Doğrulama

```bash
ros2 topic hz /ouster/points        # C++ bridge
ros2 topic hz /velodyne/points       # Python
ros2 topic hz /sbg/ros/imu/data      # Python
```

### Başlatma Sırası Özet

| Sıra | Terminal | Komut | Görevi |
|------|----------|-------|--------|
| 1 | T1 | `./CarlaUnreal.sh` | Simülasyonu başlat |
| 2 | T2 | `python3 myvehicle_control.py` | Araç spawn + kontrol |
| 3 | T3 | `python3 ...sensors... --ros --attach-only --no-ouster` | Velodyne, ZED, IMU, GNSS, TF |
| 4 | T4 | `ros2 run ros2_publisher_gcc ros2_publisher_node` | TCP server + ROS 2 Ouster |
| 5 | T5 | `./carla_reader_node` | CARLA Ouster → TCP |

### Kapatma Sırası

Ters sırada: **T5 → T4 → T3 → T2 → T1**

## Alternatif: Sadece Python (C++ olmadan)

```bash
# T1: CARLA
./CarlaUnreal.sh

# T2: Kontrol
python3 myvehicle_control.py

# T3: Tüm sensörler (Ouster dahil)
source /opt/ros/humble/setup.bash
python3 myvehicle_sensors_carla_ekstra_topic_fix.py --ros --attach-only
```

## TCP Bridge Protokolü

İki C++ proses arası iletişim custom binary paketlerle sağlanır:

```
┌──────────────────────────────────────────┐
│         PacketHeader (17 byte)           │
├────────────┬──────┬──────────┬───────────┤
│ magic (4B) │ type │ timestamp│ payload_  │
│ 0xCACA0000 │ (1B) │ (8B)     │ size (4B) │
├────────────┴──────┴──────────┴───────────┤
│         Payload (N byte)                 │
│     Ham sensör verisi (float[])          │
└──────────────────────────────────────────┘
```

`SensorType` enum: `OUSTER_LIDAR=0`, `VELODYNE_LIDAR=1`, `ZED_LEFT=2`, `ZED_RIGHT=3`, `GNSS_SBG=4`, `GNSS_REAR=5`, `IMU_SBG=6`, `IMU_2=7`, `TRANSFORM=8`