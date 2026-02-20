#!/bin/bash
set -e

# =============================================================================
# MIRACLE Workspace Build Script
# =============================================================================
# Builds the entire MIRACLE ROS2 workspace using colcon.
# Messages are built first to ensure all dependent packages can find the
# generated message/service/action headers and Python modules.
#
# Usage:
#   bash scripts/build_all.sh
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Terminal colors
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

info() { echo -e "${CYAN}[INFO]${NC}  $*"; }
error(){ echo -e "${RED}[ERROR]${NC} $*"; }
ok()  { echo -e "${GREEN}[OK]${NC}    $*"; }

# ---------------------------------------------------------------------------
# 1. Source ROS2 Jazzy base setup
# ---------------------------------------------------------------------------
ROS2_SETUP="/opt/ros/jazzy/setup.bash"

if [ ! -f "$ROS2_SETUP" ]; then
    error "ROS2 Jazzy not found. Run scripts/setup_workspace.sh first."
    exit 1
fi

info "Sourcing ROS2 Jazzy setup..."
# shellcheck disable=SC1090
source "$ROS2_SETUP"

# ---------------------------------------------------------------------------
# 2. Move to workspace root
# ---------------------------------------------------------------------------
cd "$WS_ROOT"
info "Workspace root: $WS_ROOT"

# ---------------------------------------------------------------------------
# 3. Build miracle_msgs first (message generation must precede dependents)
# ---------------------------------------------------------------------------
info "Stage 1/2: Building miracle_msgs (message generation)..."

colcon build \
    --symlink-install \
    --packages-up-to miracle_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

ok "miracle_msgs built successfully"

# Source the freshly-built messages so downstream packages can resolve them
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"

# ---------------------------------------------------------------------------
# 4. Build all remaining packages
# ---------------------------------------------------------------------------
info "Stage 2/2: Building all remaining packages..."

colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

ok "All packages built successfully"

# ---------------------------------------------------------------------------
# 5. Source the full install overlay
# ---------------------------------------------------------------------------
info "Sourcing workspace install overlay..."
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"
ok "Workspace overlay sourced"

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "============================================================"
ok "MIRACLE workspace build complete!"
echo "============================================================"
echo ""
info "To use the workspace in a new terminal:"
info "  source $WS_ROOT/install/setup.bash"
echo ""
