#!/bin/bash
set -e

# =============================================================================
# MIRACLE Workspace Test Runner
# =============================================================================
# Runs the test suites for all MIRACLE ROS2 packages and reports results.
#
# Usage:
#   bash scripts/run_tests.sh
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Terminal colors
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

info() { echo -e "${CYAN}[INFO]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error(){ echo -e "${RED}[ERROR]${NC} $*"; }
ok()  { echo -e "${GREEN}[OK]${NC}    $*"; }

# ---------------------------------------------------------------------------
# 1. Source the workspace
# ---------------------------------------------------------------------------
ROS2_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS_ROOT/install/setup.bash"

if [ ! -f "$ROS2_SETUP" ]; then
    error "ROS2 Jazzy not found. Run scripts/setup_workspace.sh first."
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS2_SETUP"

if [ -f "$WS_SETUP" ]; then
    info "Sourcing workspace overlay..."
    # shellcheck disable=SC1090
    source "$WS_SETUP"
else
    warn "Workspace not yet built -- run scripts/build_all.sh first"
    warn "Attempting to build before testing..."
    bash "$SCRIPT_DIR/build_all.sh"
    # shellcheck disable=SC1090
    source "$WS_SETUP"
fi

# ---------------------------------------------------------------------------
# 2. Define packages to test
# ---------------------------------------------------------------------------
PACKAGES=(
    miracle_core
    miracle_cnc
    miracle_scada
    miracle_bridges
    miracle_mes
    miracle_twin
    miracle_ai
    miracle_security
    miracle_resiliency
    miracle_cognitive
)

PACKAGE_LIST=$(IFS=' ' ; echo "${PACKAGES[*]}")

# ---------------------------------------------------------------------------
# 3. Run colcon test
# ---------------------------------------------------------------------------
cd "$WS_ROOT"

info "Running tests for packages: $PACKAGE_LIST"
echo ""

colcon test \
    --packages-select $PACKAGE_LIST \
    --event-handlers console_cohesion+ \
    || warn "Some tests reported failures -- see results below"

# ---------------------------------------------------------------------------
# 4. Report results
# ---------------------------------------------------------------------------
echo ""
info "Test result summary:"
echo ""

colcon test-result --verbose

echo ""
echo "============================================================"
ok "MIRACLE test run complete!"
echo "============================================================"
echo ""
