"""
lidar_accumulator.py  –  v2 (sector-based replacement)
──────────────────────────────────────────────────────
Ouster / herhangi bir LiDAR'ın kısmi taramalarını (slice A, B, C…)
sektör bazlı tamponlayıp tek bir merged PointCloud2 olarak yayınlayan
ROS 2 node.

v1 sorunu:  Aynı azimut bölgesinden gelen yeni slice eski slice'ın
            üzerine yığılıyordu → çift nokta.
v2 çözümü:  Her gelen slice'ın ortalama azimut açısından sektör ID'si
            hesaplanır.  Aynı sektör ID'sine sahip yeni veri, eskisini
            **değiştirir** (replace, append değil).

Kullanım:
  python3 lidar_accumulator.py
  # veya
  ros2 run <package> lidar_accumulator

Parametreler (ros2 param set ile değiştirilebilir):
  input_topic    : /ouster/points          (kaynak topic)
  output_topic   : /ouster/points_merged   (çıkış topic)
  window_sec     : 0.15                    (sektör verisi bu süre sonra expire olur)
  num_sectors    : 36                      (360° / 36 = 10° çözünürlük)
  publish_rate   : 20.0                    (merged cloud yayın frekansı Hz)
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2


# ─────────────────────────────────────────────────────────────────────────────
# Yardımcı fonksiyonlar
# ─────────────────────────────────────────────────────────────────────────────

def _stamp_to_float(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _newer_header(h1, h2):
    return h1 if _stamp_to_float(h1.stamp) >= _stamp_to_float(h2.stamp) else h2


def _compute_sector_key(msg: PointCloud2, num_sectors: int) -> int:
    """
    Slice'ın ortalama azimut açısını hesaplayıp [0, num_sectors) aralığında
    bir sektör ID'si döndürür.

    Böylece aynı açısal bölgeden gelen veriler hep aynı key'e düşer
    ve eskisini replace eder.
    """
    point_step = msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    n_points = len(raw) // point_step
    if n_points == 0:
        return 0

    # Hız optimizasyonu: en fazla 128 nokta örnekle (büyük cloud'lar için)
    if n_points > 128:
        step = n_points // 128
        indices = np.arange(0, n_points, step)[:128]
        block = raw.reshape(n_points, point_step)[indices]
    else:
        block = raw.reshape(n_points, point_step)

    # x (offset 0-3) ve y (offset 4-7) float32 olarak çıkar
    x = block[:, 0:4].copy().view(np.float32).ravel()
    y = block[:, 4:8].copy().view(np.float32).ravel()

    # NaN ve sıfır noktaları filtrele (self-hit → NaN, boş → 0)
    valid = np.isfinite(x) & np.isfinite(y) & ((x != 0.0) | (y != 0.0))
    if not np.any(valid):
        return 0

    mean_az = math.atan2(float(np.mean(y[valid])), float(np.mean(x[valid])))
    # atan2 → [-π, π]  →  [0, 2π) → sektör index
    sector = int((mean_az + math.pi) / (2.0 * math.pi) * num_sectors) % num_sectors
    return sector


# ─────────────────────────────────────────────────────────────────────────────
# merge_clouds: aynı point_step'e sahip N adet PointCloud2 → tek flat cloud
# ─────────────────────────────────────────────────────────────────────────────

def merge_clouds(clouds: list[PointCloud2]) -> PointCloud2 | None:
    if not clouds:
        return None

    ref = clouds[0]

    # Tüm mesajların point_step'i aynı olmalı
    if not all(c.point_step == ref.point_step for c in clouds):
        return clouds[-1]

    # data bloklarını tek seferde birleştir
    arrays = [np.frombuffer(c.data, dtype=np.uint8) for c in clouds]
    merged_data = np.concatenate(arrays)
    total_points = len(merged_data) // ref.point_step

    out = PointCloud2()
    # En güncel header'ı kullan
    best_header = ref.header
    for c in clouds[1:]:
        best_header = _newer_header(best_header, c.header)
    out.header = best_header

    # Unorganized flat cloud
    out.height = 1
    out.width = total_points
    out.fields = ref.fields
    out.is_bigendian = ref.is_bigendian
    out.point_step = ref.point_step
    out.row_step = ref.point_step * total_points
    out.is_dense = False
    out.data = merged_data.tobytes()
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Ana Node
# ─────────────────────────────────────────────────────────────────────────────

class LidarAccumulator(Node):

    def __init__(self):
        super().__init__('lidar_accumulator')

        # ── Parametreler ──────────────────────────────────────────────────
        self.declare_parameter('input_topic',  '/ouster/points')
        self.declare_parameter('output_topic', '/ouster/points_merged')
        self.declare_parameter('window_sec',   0.15)    # sektör expire süresi
        self.declare_parameter('num_sectors',  36)      # 360°/36 = 10° çözünürlük
        self.declare_parameter('publish_rate', 20.0)    # Hz

        in_topic    = self.get_parameter('input_topic').value
        out_topic   = self.get_parameter('output_topic').value
        self._win   = self.get_parameter('window_sec').value
        self._nsec  = self.get_parameter('num_sectors').value
        pub_rate    = self.get_parameter('publish_rate').value

        # ── Sektör tabanlı buffer ─────────────────────────────────────────
        # Key: sektör ID (0 .. num_sectors-1)
        # Val: (arrival_monotonic, PointCloud2)
        # Aynı sektöre yeni veri gelince eskisi overwrite olur → çift nokta yok
        self._sectors: dict[int, tuple[float, PointCloud2]] = {}
        self._lock = threading.Lock()

        # ── İstatistik (debug log) ────────────────────────────────────────
        self._stats_updates = 0
        self._stats_replaces = 0

        # ── QoS ──────────────────────────────────────────────────────────
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Sub / Pub ─────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            PointCloud2, in_topic, self._cb, sub_qos
        )
        self._pub = self.create_publisher(PointCloud2, out_topic, pub_qos)

        # ── Yayın timer'ı ────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / pub_rate, self._publish)

        # ── Debug log timer (her 5 saniyede durum yazdır) ─────────────────
        self._log_timer = self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"LidarAccumulator v2 (sector-replace) başlatıldı\n"
            f"  {in_topic} → {out_topic}\n"
            f"  window={self._win}s  sectors={self._nsec}  rate={pub_rate}Hz\n"
            f"  Aynı sektöre yeni veri gelince eski veri DEĞİŞTİRİLİR (stack değil)"
        )

    # ── Callback: sektör hesapla → replace ────────────────────────────────
    def _cb(self, msg: PointCloud2):
        sector = _compute_sector_key(msg, self._nsec)
        now = time.monotonic()

        with self._lock:
            was_replace = sector in self._sectors
            self._sectors[sector] = (now, msg)

            self._stats_updates += 1
            if was_replace:
                self._stats_replaces += 1

    # ── Timer: expire + merge + publish ───────────────────────────────────
    def _publish(self):
        now = time.monotonic()
        cutoff = now - self._win

        with self._lock:
            # Zaman penceresi dışındaki sektörleri sil
            expired = [k for k, (t, _) in self._sectors.items() if t < cutoff]
            for k in expired:
                del self._sectors[k]

            # Kalan sektörlerin mesajlarını al
            clouds = [msg for (_, msg) in self._sectors.values()]

        if not clouds:
            return

        merged = merge_clouds(clouds)
        if merged is not None:
            self._pub.publish(merged)

    # ── Debug log ─────────────────────────────────────────────────────────
    def _log_stats(self):
        with self._lock:
            n_active = len(self._sectors)
            total = self._stats_updates
            replaces = self._stats_replaces
            # Reset counters
            self._stats_updates = 0
            self._stats_replaces = 0

        if total > 0:
            self.get_logger().info(
                f"[stats] active_sectors={n_active}/{self._nsec}  "
                f"updates={total}  replaces={replaces}  "
                f"(replace oranı: {100*replaces/total:.0f}%)"
            )


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = LidarAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
