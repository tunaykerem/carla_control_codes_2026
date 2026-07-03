#!/usr/bin/env bash
# ============================================================================
#  launch_all.sh — Tüm CARLA + ROS 2 + C++ Bridge sistemini tek seferde başlatır
#
#  Kullanım:
#    ./launch_all.sh              # Tüm sistemi başlat (5 tmux pane)
#    ./launch_all.sh --no-cpp     # Sadece Python (C++ bridge olmadan)
#    ./launch_all.sh --kill       # Oturumu kapat
#
#  Gereksinim: tmux
#
#  Başlatma sırası (otomatik):
#    1. CARLA Simülasyon
#    2. Python Araç Kontrolü (myvehicle_control.py)
#    3. Python Sensörler (--no-ouster --no-zed)
#    4. ROS 2 Publisher (ros2_publisher_gcc)
#    5. CARLA Reader (carla_reader_clang)
#
#  Kapatma:
#    ./launch_all.sh --kill   veya   tmux kill-session -t carla
# ============================================================================

set -e

# ── Renkler ─────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ── Sabitler ────────────────────────────────────────────────────────────────
SESSION="carla"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
CARLA_DIR="${CARLA_SIM_DIR:-}"
BRIDGE_DIR="$PROJECT_DIR/carla_sensor_bridge"
READER_BIN="$BRIDGE_DIR/carla_reader_clang/build/carla_reader_node"
ROS_SETUP="source /opt/ros/humble/setup.bash"
BRIDGE_SETUP="source $BRIDGE_DIR/install/setup.bash"

# ── Bekleme süreleri (saniye) ───────────────────────────────────────────────
WAIT_CARLA=20        # CARLA'nın başlamasını bekle
WAIT_CONTROL=8       # Aracın spawn olmasını bekle
WAIT_SENSORS=5       # Python sensörlerin başlamasını bekle
WAIT_PUBLISHER=3     # TCP server'ın ayağa kalkmasını bekle

# ── Argüman parse ───────────────────────────────────────────────────────────
NO_CPP=false

for arg in "$@"; do
    case "$arg" in
        --kill)
            echo -e "${YELLOW}[KILL]${NC} Oturum kapatılıyor: ${SESSION}"
            tmux kill-session -t "$SESSION" 2>/dev/null && \
                echo -e "${GREEN}[OK]${NC} Oturum kapatıldı." || \
                echo -e "${RED}[HATA]${NC} Aktif '${SESSION}' oturumu bulunamadı."
            exit 0
            ;;
        --no-cpp)
            NO_CPP=true
            ;;
        --help|-h)
            echo "Kullanım: $0 [--no-cpp] [--kill] [--help]"
            echo ""
            echo "  --no-cpp    C++ bridge olmadan sadece Python ile başlat"
            echo "  --kill      Mevcut carla tmux oturumunu kapat"
            echo "  --help      Bu yardım mesajını göster"
            exit 0
            ;;
        *)
            echo -e "${RED}[HATA]${NC} Bilinmeyen argüman: $arg"
            echo "Kullanım: $0 [--no-cpp] [--kill] [--help]"
            exit 1
            ;;
    esac
done

# ── Ön kontrol: tmux ───────────────────────────────────────────────────────
if ! command -v tmux &>/dev/null; then
    echo -e "${RED}[HATA]${NC} tmux bulunamadı! Yüklemek için:"
    echo -e "  ${YELLOW}sudo apt install tmux${NC}"
    exit 1
fi

# ── Ön kontrol: CARLA simülasyon dizini ────────────────────────────────────
if [[ -z "$CARLA_DIR" ]]; then
    # Auto-detect: bilinen konumları tara
    for candidate in \
        "$HOME/Carla-0.10.0-Linux-Shipping/Linux" \
        "$HOME/carla/Carla-0.10.0-Linux-Shipping/Linux" \
        "/opt/carla/Carla-0.10.0-Linux-Shipping/Linux"; do
        if [[ -f "$candidate/CarlaUnreal.sh" ]]; then
            CARLA_DIR="$candidate"
            echo -e "${GREEN}[AUTO-DETECT]${NC} CARLA simülasyon bulundu: ${CARLA_DIR}"
            break
        fi
    done
fi

if [[ -z "$CARLA_DIR" || ! -f "$CARLA_DIR/CarlaUnreal.sh" ]]; then
    echo -e "${RED}[HATA]${NC} CARLA simülasyon dizini bulunamadı!"
    echo -e "  ${YELLOW}CARLA_SIM_DIR${NC} ortam değişkenini ayarlayın:"
    echo -e "  ${CYAN}export CARLA_SIM_DIR=/path/to/Carla-0.10.0-Linux-Shipping/Linux${NC}"
    echo -e "  Dizin içinde ${CYAN}CarlaUnreal.sh${NC} dosyası bulunmalıdır."
    exit 1
fi

# ── Ön kontrol: mevcut oturum ──────────────────────────────────────────────
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo -e "${YELLOW}[UYARI]${NC} '${SESSION}' adlı bir tmux oturumu zaten var."
    echo -e "  Önce kapatmak için: ${CYAN}./launch_all.sh --kill${NC}"
    echo -e "  Bağlanmak için:     ${CYAN}tmux attach -t ${SESSION}${NC}"
    exit 1
fi

# ── Ön kontrol: Build dosyaları ─────────────────────────────────────────────
if [[ "$NO_CPP" == false ]]; then
    if [[ ! -f "$READER_BIN" ]]; then
        echo -e "${RED}[HATA]${NC} carla_reader_node bulunamadı: ${READER_BIN}"
        echo -e "  Build etmek için:"
        echo -e "  ${YELLOW}cd $BRIDGE_DIR/carla_reader_clang && mkdir -p build && cd build${NC}"
        echo -e "  ${YELLOW}CC=clang CXX=clang++ cmake .. && make -j\$(nproc)${NC}"
        exit 1
    fi

    if [[ ! -d "$BRIDGE_DIR/install/ros2_publisher_gcc" ]]; then
        echo -e "${RED}[HATA]${NC} ros2_publisher_gcc install bulunamadı."
        echo -e "  Build etmek için:"
        echo -e "  ${YELLOW}cd $BRIDGE_DIR && colcon build --packages-select ros2_publisher_gcc${NC}"
        exit 1
    fi
fi

# ════════════════════════════════════════════════════════════════════════════
#  BAŞLATMA
# ════════════════════════════════════════════════════════════════════════════

echo ""
echo -e "${GREEN}${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}${BOLD}║       🚗  CARLA Hibrit Sistem — Tek Seferde Başlatma        ║${NC}"
echo -e "${GREEN}${BOLD}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

if [[ "$NO_CPP" == true ]]; then
    echo -e "${CYAN}[MOD]${NC} Sadece Python modu (C++ bridge devre dışı)"
    echo ""
fi

# ── Pane isimlendirme yardımcısı ────────────────────────────────────────────
# tmux pane başlıklarını renkli yapmak için her pane'de bir banner basalım
banner() {
    local color=$1 icon=$2 title=$3
    echo "echo -e '${color}${BOLD}═══════════════════════════════════════════${NC}'"
    echo "echo -e '${color}${BOLD}  ${icon}  ${title}${NC}'"
    echo "echo -e '${color}${BOLD}═══════════════════════════════════════════${NC}'"
    echo "echo ''"
}

# ────────────────────────────────────────────────────────────────────────────
# Terminal 1: CARLA Simülasyon
# ────────────────────────────────────────────────────────────────────────────
echo -e "${GREEN}[1/5]${NC} 🎮 CARLA Simülasyon başlatılıyor..."

tmux new-session -d -s "$SESSION" -n "carla" \
    "$(banner '\033[0;32m' '🎮' 'Terminal 1 — CARLA Simülasyon')
     cd $CARLA_DIR && ./CarlaUnreal.sh; bash"

# CARLA'nın başlamasını bekle
echo -e "      ${CYAN}⏳ CARLA'nın başlaması bekleniyor (${WAIT_CARLA}s)...${NC}"
sleep "$WAIT_CARLA"

# ────────────────────────────────────────────────────────────────────────────
# Terminal 2: Python Araç Kontrolü
# ────────────────────────────────────────────────────────────────────────────
echo -e "${GREEN}[2/5]${NC} 🐍 Araç kontrolü başlatılıyor..."

tmux new-window -t "$SESSION" -n "control" \
    "$(banner '\033[1;33m' '🐍' 'Terminal 2 — Araç Kontrolü (myvehicle_control.py)')
     cd $PROJECT_DIR && python3 myvehicle_control.py; bash"

# Aracın spawn olmasını bekle
echo -e "      ${CYAN}⏳ Aracın spawn olması bekleniyor (${WAIT_CONTROL}s)...${NC}"
sleep "$WAIT_CONTROL"

# ────────────────────────────────────────────────────────────────────────────
# Terminal 3: Python Sensörler
# ────────────────────────────────────────────────────────────────────────────
echo -e "${GREEN}[3/5]${NC} 📡 Python sensörler başlatılıyor..."

if [[ "$NO_CPP" == true ]]; then
    SENSOR_CMD="python3 myvehicle_sensors_carla_ekstra_topic_fix.py --ros --attach-only"
    SENSOR_TITLE="Terminal 3 — Python Sensörler (TÜM sensörler)"
else
    SENSOR_CMD="python3 myvehicle_sensors_carla_ekstra_topic_fix.py --ros --attach-only --no-ouster --no-zed"
    SENSOR_TITLE="Terminal 3 — Python Sensörler (--no-ouster --no-zed)"
fi

tmux new-window -t "$SESSION" -n "sensors" \
    "$(banner '\033[0;36m' '📡' "$SENSOR_TITLE")
     $ROS_SETUP && cd $PROJECT_DIR && $SENSOR_CMD; bash"

echo -e "      ${CYAN}⏳ Sensörlerin başlaması bekleniyor (${WAIT_SENSORS}s)...${NC}"
sleep "$WAIT_SENSORS"

# ────────────────────────────────────────────────────────────────────────────
# Terminal 4 & 5: C++ Bridge (sadece --no-cpp değilse)
# ────────────────────────────────────────────────────────────────────────────
if [[ "$NO_CPP" == false ]]; then
    # Terminal 4: ROS 2 Publisher
    echo -e "${GREEN}[4/5]${NC} 🤖 ROS 2 Publisher başlatılıyor..."

    tmux new-window -t "$SESSION" -n "publisher" \
        "$(banner '\033[0;35m' '🤖' 'Terminal 4 — ROS 2 Publisher (ros2_publisher_gcc)')
         $ROS_SETUP && $BRIDGE_SETUP && ros2 run ros2_publisher_gcc ros2_publisher_node; bash"

    echo -e "      ${CYAN}⏳ TCP server'ın başlaması bekleniyor (${WAIT_PUBLISHER}s)...${NC}"
    sleep "$WAIT_PUBLISHER"

    # Terminal 5: CARLA Reader
    echo -e "${GREEN}[5/5]${NC} ⚡ CARLA Reader başlatılıyor..."

    tmux new-window -t "$SESSION" -n "reader" \
        "$(banner '\033[0;31m' '⚡' 'Terminal 5 — CARLA Reader (carla_reader_clang)')
         cd $BRIDGE_DIR/carla_reader_clang/build && ./carla_reader_node; bash"

else
    echo -e "${YELLOW}[4/5]${NC} ⏭  C++ Publisher atlandı (--no-cpp)"
    echo -e "${YELLOW}[5/5]${NC} ⏭  C++ Reader atlandı (--no-cpp)"
fi

# ════════════════════════════════════════════════════════════════════════════
#  TAMAMLANDI
# ════════════════════════════════════════════════════════════════════════════

echo ""
echo -e "${GREEN}${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}${BOLD}║                    ✅  Sistem Hazır!                        ║${NC}"
echo -e "${GREEN}${BOLD}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  ${CYAN}Bağlanmak için:${NC}"
echo -e "    ${YELLOW}tmux attach -t ${SESSION}${NC}"
echo ""
echo -e "  ${CYAN}Pencereler arası geçiş (tmux içinde):${NC}"
echo -e "    ${YELLOW}Ctrl+B → sayı tuşu (0-4)${NC}     belirli pencereye git"
echo -e "    ${YELLOW}Ctrl+B → n${NC}                    sonraki pencere"
echo -e "    ${YELLOW}Ctrl+B → p${NC}                    önceki pencere"
echo -e "    ${YELLOW}Ctrl+B → w${NC}                    pencere listesi"
echo ""
echo -e "  ${CYAN}Kapatmak için:${NC}"
echo -e "    ${YELLOW}./launch_all.sh --kill${NC}        tüm sistemi kapat"
echo -e "    ${YELLOW}tmux kill-session -t ${SESSION}${NC}  aynı şey"
echo ""
echo -e "  ${CYAN}Pencere düzeni:${NC}"
echo -e "    ${GREEN}0:carla${NC}     — 🎮 CARLA Simülasyon"
echo -e "    ${GREEN}1:control${NC}   — 🐍 Araç Kontrolü"
echo -e "    ${GREEN}2:sensors${NC}   — 📡 Python Sensörler"
if [[ "$NO_CPP" == false ]]; then
    echo -e "    ${GREEN}3:publisher${NC} — 🤖 ROS 2 Publisher"
    echo -e "    ${GREEN}4:reader${NC}    — ⚡ CARLA Reader"
fi
echo ""
