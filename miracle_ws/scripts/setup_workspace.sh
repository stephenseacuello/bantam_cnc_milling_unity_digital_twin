#!/bin/bash
set -e

# =============================================================================
# MIRACLE Workspace Setup Script
# =============================================================================
# Sets up the MIRACLE (Manufacturing Integrated Resilient Adaptive Control
# Loop Environment) ROS2 workspace with all required dependencies.
#
# Usage:
#   bash scripts/setup_workspace.sh [--venv]
#
# Options:
#   --venv    Create and activate a Python virtual environment
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Terminal colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }

# ---------------------------------------------------------------------------
# 1. Check for ROS2 Jazzy installation
# ---------------------------------------------------------------------------
info "Checking for ROS2 Jazzy installation..."

ROS2_SETUP="/opt/ros/jazzy/setup.bash"

if [ ! -f "$ROS2_SETUP" ]; then
    error "ROS2 Jazzy not found at $ROS2_SETUP"
    error "Please install ROS2 Jazzy first:"
    error "  https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

ok "ROS2 Jazzy found at $ROS2_SETUP"

# Source ROS2 base setup
# shellcheck disable=SC1090
source "$ROS2_SETUP"
ok "ROS2 Jazzy environment sourced"

# ---------------------------------------------------------------------------
# 2. (Optional) Create Python virtual environment
# ---------------------------------------------------------------------------
USE_VENV=false
for arg in "$@"; do
    if [ "$arg" = "--venv" ]; then
        USE_VENV=true
    fi
done

if [ "$USE_VENV" = true ]; then
    VENV_DIR="$WS_ROOT/.venv"
    info "Creating Python virtual environment at $VENV_DIR ..."

    if [ ! -d "$VENV_DIR" ]; then
        python3 -m venv "$VENV_DIR" --system-site-packages
        ok "Virtual environment created"
    else
        warn "Virtual environment already exists at $VENV_DIR -- reusing"
    fi

    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
    ok "Virtual environment activated"
fi

# ---------------------------------------------------------------------------
# 3. Install Python dependencies (core)
# ---------------------------------------------------------------------------
info "Installing core Python dependencies..."

pip3 install --quiet --upgrade pip

pip3 install --quiet \
    numpy \
    scipy \
    scikit-learn

ok "Core Python dependencies installed (numpy, scipy, scikit-learn)"

# ---------------------------------------------------------------------------
# 4. Install optional Python dependencies (industrial protocol bridges)
# ---------------------------------------------------------------------------
info "Installing optional Python dependencies (industrial protocols)..."

pip3 install --quiet \
    opcua \
    paho-mqtt \
    pymodbus \
    kafka-python \
    || warn "Some optional dependencies failed to install -- non-fatal"

ok "Optional Python dependencies installed (opcua, paho-mqtt, pymodbus, kafka-python)"

# ---------------------------------------------------------------------------
# 5. Source ROS2 setup (ensure colcon / rosdep are available)
# ---------------------------------------------------------------------------
info "Verifying ROS2 tooling..."

if ! command -v colcon &> /dev/null; then
    warn "colcon not found on PATH -- installing python3-colcon-common-extensions"
    sudo apt-get update -qq && sudo apt-get install -y -qq python3-colcon-common-extensions
fi

if ! command -v rosdep &> /dev/null; then
    warn "rosdep not found on PATH -- installing python3-rosdep"
    sudo apt-get update -qq && sudo apt-get install -y -qq python3-rosdep
fi

# ---------------------------------------------------------------------------
# 6. Run rosdep install
# ---------------------------------------------------------------------------
info "Running rosdep install for workspace packages..."

# Initialise rosdep if it hasn't been done yet (first-time setup)
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
    info "Initialising rosdep for the first time..."
    sudo rosdep init || warn "rosdep init may have already been run"
fi
rosdep update --rosdistro=jazzy || warn "rosdep update encountered warnings"

rosdep install \
    --from-paths "$WS_ROOT/src" \
    --ignore-src \
    --rosdistro=jazzy \
    -y \
    || warn "Some rosdep keys could not be resolved -- check output above"

ok "rosdep install complete"

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "============================================================"
ok "MIRACLE workspace setup complete!"
echo "============================================================"
echo ""
info "Next steps:"
info "  1. Build the workspace:   bash scripts/build_all.sh"
info "  2. Run tests:             bash scripts/run_tests.sh"
echo ""
