#!/bin/bash
set -e

# =============================================================================
# MIRACLE Workspace Deployment Helper
# =============================================================================
# Deploys the MIRACLE ROS2 workspace to a remote target host via rsync + SSH.
#
# Usage:
#   bash scripts/deploy.sh <user@host> [--remote-dir /path/on/target]
#
# Examples:
#   bash scripts/deploy.sh operator@192.168.1.100
#   bash scripts/deploy.sh operator@cnc-edge-01 --remote-dir /opt/miracle_ws
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Defaults
REMOTE_DIR="/opt/miracle_ws"

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
# 1. Parse arguments
# ---------------------------------------------------------------------------
if [ $# -lt 1 ]; then
    error "Usage: bash scripts/deploy.sh <user@host> [--remote-dir /path]"
    exit 1
fi

TARGET_HOST="$1"
shift

while [ $# -gt 0 ]; do
    case "$1" in
        --remote-dir)
            REMOTE_DIR="$2"
            shift 2
            ;;
        *)
            warn "Unknown argument: $1"
            shift
            ;;
    esac
done

info "Target host:       $TARGET_HOST"
info "Remote directory:  $REMOTE_DIR"
echo ""

# ---------------------------------------------------------------------------
# 2. Pre-flight checks
# ---------------------------------------------------------------------------
if ! command -v rsync &> /dev/null; then
    error "rsync is not installed. Please install rsync first."
    exit 1
fi

if ! command -v ssh &> /dev/null; then
    error "ssh is not installed. Please install openssh-client first."
    exit 1
fi

# Verify SSH connectivity
info "Verifying SSH connectivity to $TARGET_HOST ..."
if ! ssh -o ConnectTimeout=10 -o BatchMode=yes "$TARGET_HOST" "echo ok" &> /dev/null; then
    warn "SSH connection test failed. You may be prompted for a password."
fi

# ---------------------------------------------------------------------------
# 3. Sync workspace to target host
# ---------------------------------------------------------------------------
info "Syncing workspace to $TARGET_HOST:$REMOTE_DIR ..."

# Create the remote directory if it does not exist
ssh "$TARGET_HOST" "mkdir -p $REMOTE_DIR"

rsync -avz \
    --progress \
    --delete \
    --exclude '.venv/' \
    --exclude 'build/' \
    --exclude 'install/' \
    --exclude 'log/' \
    --exclude '.git/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    "$WS_ROOT/" \
    "$TARGET_HOST:$REMOTE_DIR/"

ok "Workspace synced to $TARGET_HOST:$REMOTE_DIR"

# ---------------------------------------------------------------------------
# 4. Run setup and build on the remote target
# ---------------------------------------------------------------------------
info "Running setup and build on remote target..."

ssh -t "$TARGET_HOST" bash <<REMOTE_EOF
set -e
cd "$REMOTE_DIR"

echo "[REMOTE] Running workspace setup..."
bash scripts/setup_workspace.sh

echo "[REMOTE] Building workspace..."
bash scripts/build_all.sh

echo "[REMOTE] Setup and build complete."
REMOTE_EOF

ok "Remote setup and build finished"

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "============================================================"
ok "MIRACLE deployment to $TARGET_HOST complete!"
echo "============================================================"
echo ""
info "To launch on the remote target:"
info "  ssh $TARGET_HOST"
info "  cd $REMOTE_DIR"
info "  source install/setup.bash"
info "  ros2 launch miracle_bringup miracle_full.launch.py"
echo ""
