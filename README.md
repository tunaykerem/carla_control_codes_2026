# carla_control_codes_2026

CARLA Simulator için Python tabanlı araç kontrol kodları.  
Python-based vehicle control scripts for the [CARLA Simulator](https://carla.org/).

---

## İçerik / Contents

| Dosya | Açıklama |
|-------|----------|
| `manual_control.py` | Klavye ile araç kontrolü (pygame) |
| `pid_controller.py` | Tekrar kullanılabilir PID boyuna / yanal kontrolcü |
| `waypoint_follower.py` | PID tabanlı waypoint takip ajanı |
| `spawn_vehicles.py` | NPC trafik araçlarını simülasyona ekler |
| `requirements.txt` | Python bağımlılıkları |

---

## Gereksinimler / Requirements

* CARLA ≥ 0.9.14 (sunucu çalışır durumda olmalı)
* Python 3.8+

```bash
pip install -r requirements.txt
```

---

## Kullanım / Usage

### Sunucuyu Başlat / Start the Server
```bash
./CarlaUE4.sh -quality-level=Low
```

### Manuel Kontrol / Manual Control
```bash
python manual_control.py
```
Klavye kısayolları:
* **W / S** – gaz / fren
* **A / D** – sol / sağ
* **Q** – geri vites
* **R** – trafik ışıklarını sıfırla
* **ESC** – çıkış

### Waypoint Takip Ajanı / Waypoint Follower
```bash
python waypoint_follower.py --host 127.0.0.1 --port 2000 --speed 30
```

### NPC Araç Spawn / Spawn NPC Vehicles
```bash
python spawn_vehicles.py --n 50
```

---

## Lisans / License

MIT
