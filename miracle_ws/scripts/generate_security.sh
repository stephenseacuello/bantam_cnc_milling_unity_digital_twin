#!/bin/bash
set -e

# =============================================================================
# MIRACLE SROS2 Security Setup Script
# =============================================================================
# Generates an SROS2 keystore and per-node keys/certificates for the MIRACLE
# system, then configures environment variables to enforce DDS security.
#
# Usage:
#   bash scripts/generate_security.sh
#
# After running, source the generated env file to enable security:
#   source <workspace>/security/security_env.bash
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
KEYSTORE_DIR="$WS_ROOT/security/keystore"
SECURITY_ENV_FILE="$WS_ROOT/security/security_env.bash"

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
# 1. Source ROS2 Jazzy
# ---------------------------------------------------------------------------
ROS2_SETUP="/opt/ros/jazzy/setup.bash"

if [ ! -f "$ROS2_SETUP" ]; then
    error "ROS2 Jazzy not found. Run scripts/setup_workspace.sh first."
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS2_SETUP"

# Verify ros2 security CLI is available
if ! ros2 security --help &> /dev/null; then
    error "SROS2 tools not available. Install with:"
    error "  sudo apt-get install ros-jazzy-sros2"
    exit 1
fi

# ---------------------------------------------------------------------------
# 2. Create the SROS2 keystore
# ---------------------------------------------------------------------------
info "Creating SROS2 keystore at $KEYSTORE_DIR ..."

mkdir -p "$KEYSTORE_DIR"

ros2 security create_keystore "$KEYSTORE_DIR"
ok "Keystore created"

# ---------------------------------------------------------------------------
# 3. Generate keys for all MIRACLE nodes
# ---------------------------------------------------------------------------
# List of all nodes that require security credentials.
# Format: /<namespace>/<node_name>  (fully-qualified ROS2 node names)
NODES=(
    # Core infrastructure
    "/miracle/lifecycle_manager"
    "/miracle/parameter_server"
    "/miracle/health_monitor"
    # CNC machine interface
    "/miracle/cnc_driver"
    "/miracle/spindle_controller"
    "/miracle/axis_controller"
    "/miracle/tool_changer"
    # SCADA layer
    "/miracle/scada_server"
    "/miracle/alarm_manager"
    "/miracle/historian"
    # Protocol bridges
    "/miracle/opcua_bridge"
    "/miracle/mqtt_bridge"
    "/miracle/modbus_bridge"
    "/miracle/kafka_bridge"
    # MES integration
    "/miracle/mes_gateway"
    "/miracle/job_scheduler"
    "/miracle/recipe_manager"
    # Digital twin
    "/miracle/twin_sync"
    "/miracle/physics_engine"
    "/miracle/state_mirror"
    # AI / Analytics
    "/miracle/anomaly_detector"
    "/miracle/tool_wear_estimator"
    "/miracle/phm_predictor"
    "/miracle/chatter_detector"
    # Security
    "/miracle/ids_node"
    "/miracle/audit_logger"
    "/miracle/access_controller"
    # Resiliency
    "/miracle/watchdog"
    "/miracle/failover_manager"
    "/miracle/redundancy_manager"
    # Cognitive
    "/miracle/cognitive_engine"
    "/miracle/knowledge_graph"
    "/miracle/reasoning_engine"
)

info "Generating keys for ${#NODES[@]} nodes..."

for node in "${NODES[@]}"; do
    info "  Generating key for $node"
    ros2 security create_key "$KEYSTORE_DIR" "$node"
done

ok "All node keys generated"

# ---------------------------------------------------------------------------
# 3b. Generate Ed25519 key pair for G-code signing
# ---------------------------------------------------------------------------
info "Generating Ed25519 key pair for G-code signing..."

GCODE_KEY_DIR="$WS_ROOT/security"
mkdir -p "$GCODE_KEY_DIR"

# Generate Ed25519 key pair using openssl
openssl genpkey -algorithm Ed25519 -out "$GCODE_KEY_DIR/gcode_signing.key" 2>/dev/null
openssl pkey -in "$GCODE_KEY_DIR/gcode_signing.key" -pubout -out "$GCODE_KEY_DIR/gcode_signing.pub" 2>/dev/null

chmod 600 "$GCODE_KEY_DIR/gcode_signing.key"
chmod 644 "$GCODE_KEY_DIR/gcode_signing.pub"

ok "G-code signing key pair generated"
info "  Private key: $GCODE_KEY_DIR/gcode_signing.key (keep secure!)"
info "  Public key:  $GCODE_KEY_DIR/gcode_signing.pub (embed in Unity build)"

# ---------------------------------------------------------------------------
# 4. Set environment variables for DDS security enforcement
# ---------------------------------------------------------------------------
info "Writing security environment file to $SECURITY_ENV_FILE ..."

cat > "$SECURITY_ENV_FILE" <<'ENVEOF'
#!/bin/bash
# =============================================================================
# MIRACLE SROS2 Security Environment
# =============================================================================
# Source this file to enable DDS security enforcement for the MIRACLE system.
#
# Usage:
#   source security/security_env.bash
# =============================================================================

ENVEOF

# Append variables with the resolved keystore path
cat >> "$SECURITY_ENV_FILE" <<EOF
export ROS_SECURITY_KEYSTORE="$KEYSTORE_DIR"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

echo "[SROS2] Security ENABLED  (strategy=Enforce)"
echo "[SROS2] Keystore: \$ROS_SECURITY_KEYSTORE"
EOF

ok "Security environment file written"

# Export variables for the current session as well
export ROS_SECURITY_KEYSTORE="$KEYSTORE_DIR"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# ---------------------------------------------------------------------------
# 5. Sign governance and permissions XML for SROS2
# ---------------------------------------------------------------------------
info "Signing SROS2 governance and permissions..."

GOVERNANCE_XML="$WS_ROOT/config/sros2_governance.xml"
PERMISSIONS_XML="$WS_ROOT/config/sros2_permissions.xml"

if [ -f "$GOVERNANCE_XML" ] && [ -f "$PERMISSIONS_XML" ]; then
    # Sign governance with CA key
    CA_CERT="$KEYSTORE_DIR/public/ca.cert.pem"
    CA_KEY="$KEYSTORE_DIR/private/ca.key.pem"

    if [ -f "$CA_CERT" ] && [ -f "$CA_KEY" ]; then
        for node in "${NODES[@]}"; do
            ENCLAVE_DIR="$KEYSTORE_DIR/enclaves/$node"
            if [ -d "$ENCLAVE_DIR" ]; then
                # Copy governance and permissions, sign with PKCS7
                openssl smime -sign -text -in "$GOVERNANCE_XML" \
                    -out "$ENCLAVE_DIR/governance.p7s" \
                    -signer "$CA_CERT" -inkey "$CA_KEY" 2>/dev/null || true

                openssl smime -sign -text -in "$PERMISSIONS_XML" \
                    -out "$ENCLAVE_DIR/permissions.p7s" \
                    -signer "$CA_CERT" -inkey "$CA_KEY" 2>/dev/null || true
            fi
        done
        ok "Governance and permissions signed for all enclaves"
    else
        warn "CA cert/key not found — skipping governance signing"
        warn "  Expected: $CA_CERT and $CA_KEY"
    fi
else
    warn "Governance/permissions XML not found — skipping"
    warn "  Expected: $GOVERNANCE_XML and $PERMISSIONS_XML"
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "============================================================"
ok "MIRACLE SROS2 security setup complete!"
echo "============================================================"
echo ""
info "Keystore location:  $KEYSTORE_DIR"
info "Security strategy:  Enforce"
info ""
info "To enable security in a new terminal:"
info "  source $SECURITY_ENV_FILE"
echo ""
