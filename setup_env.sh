#!/usr/bin/env bash
# ============================================================================
#  setup_env.sh  —  Portable environment setup for carla_control_codes_2026
#
#  Usage:
#    source setup_env.sh                          # auto-detect CARLA_ROOT
#    source setup_env.sh /path/to/CarlaUE5        # explicit CARLA_ROOT
#
#  This script MUST be sourced (not executed) so that the exported variables
#  persist in your shell session.
# ============================================================================

set -e

# ── Color helpers ───────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ── Determine CARLA_ROOT ───────────────────────────────────────────────────
if [[ -n "$1" ]]; then
    # User passed an explicit path as argument
    CARLA_ROOT="$1"
elif [[ -n "$CARLA_ROOT" ]]; then
    # Already set in environment
    echo -e "${CYAN}[INFO]${NC} Using existing CARLA_ROOT from environment: ${CARLA_ROOT}"
else
    # Auto-detect: search common locations
    SEARCH_PATHS=(
        "$HOME/CarlaUE5"
        "$HOME/carla/CarlaUE5"
        "$HOME/itu_sct_gae/karla_zimulazyon/CarlaUE5"
        "/opt/carla/CarlaUE5"
        "/opt/CarlaUE5"
    )

    for candidate in "${SEARCH_PATHS[@]}"; do
        if [[ -d "$candidate/LibCarla/source" ]]; then
            CARLA_ROOT="$candidate"
            echo -e "${GREEN}[AUTO-DETECT]${NC} Found CARLA at: ${CARLA_ROOT}"
            break
        fi
    done

    if [[ -z "$CARLA_ROOT" ]]; then
        echo -e "${RED}══════════════════════════════════════════════════════════════════${NC}"
        echo -e "${RED}  CARLA_ROOT could not be auto-detected!${NC}"
        echo ""
        echo -e "  Please provide the path to your CarlaUE5 directory:"
        echo ""
        echo -e "    ${YELLOW}source setup_env.sh /path/to/CarlaUE5${NC}"
        echo ""
        echo -e "  Or set it manually:"
        echo ""
        echo -e "    ${YELLOW}export CARLA_ROOT=/path/to/CarlaUE5${NC}"
        echo -e "${RED}══════════════════════════════════════════════════════════════════${NC}"
        return 1 2>/dev/null || exit 1
    fi
fi

# ── Validate ────────────────────────────────────────────────────────────────
if [[ ! -d "$CARLA_ROOT/LibCarla/source" ]]; then
    echo -e "${RED}[ERROR]${NC} CARLA_ROOT is set to: ${CARLA_ROOT}"
    echo -e "${RED}        but '${CARLA_ROOT}/LibCarla/source' does not exist.${NC}"
    echo -e "        Please check that the path is correct and CARLA is built."
    return 1 2>/dev/null || exit 1
fi

# ── Export ──────────────────────────────────────────────────────────────────
export CARLA_ROOT="$CARLA_ROOT"

echo -e "${GREEN}══════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  Environment configured successfully!${NC}"
echo ""
echo -e "  CARLA_ROOT = ${CYAN}${CARLA_ROOT}${NC}"
echo ""
echo -e "  You can now build the projects:"
echo ""
echo -e "  ${YELLOW}# carla_reader_clang (standalone, uses clang/libc++)${NC}"
echo -e "  cd carla_sensor_bridge/carla_reader_clang"
echo -e "  mkdir -p build && cd build"
echo -e "  CC=clang CXX=clang++ cmake .."
echo -e "  make -j\$(nproc)"
echo ""
echo -e "  ${YELLOW}# carla_sensor_publisher (ROS 2, uses colcon)${NC}"
echo -e "  cd <workspace_root>"
echo -e "  colcon build --packages-select carla_sensor_publisher"
echo -e "${GREEN}══════════════════════════════════════════════════════════════════${NC}"
