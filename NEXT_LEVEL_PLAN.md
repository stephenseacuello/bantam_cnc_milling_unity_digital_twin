# MIRACLE Digital Twin — Next-Level Improvement Plan

## Domains: Digital Twin Fidelity, Situational Awareness, Security & Resiliency

### Current State

The system already has: Altintas mechanistic cutting forces, GPU voxel subtraction, thermal ODE, 3-stage Taylor tool wear, 50Hz bidirectional ROS2 sync, ISA-18.2 alarm manager, multi-model anomaly ensemble (Z-score + IF + PCA), SHA256-chained digital thread, full replay system, RBAC access control, IDS with rate/burst/payload detection, circuit breaker pattern, and K8s network policies with default-deny.

### What's Missing

The twin can't predict ahead. Alerts aren't correlated. Traffic is unencrypted. The prediction runner returns hardcoded values. Recovery orchestration is stubbed. G-code programs aren't signed. There's no closed-loop feedback from digital to physical. This plan closes those gaps.

---

## Phase 1: Predictive Twin + Security Foundation

> Goal: The twin simulates ahead of the physical machine and critical control paths are encrypted/signed.

### 1.1 — Real Prediction Runner

**Problem:** `prediction_runner.py` returns hardcoded RUL=450h, conf=0.87. It doesn't run any actual simulation.

**Files:**
- Modify: `miracle_ws/src/miracle_twin/miracle_twin/prediction_runner.py`
- Modify: `miracle_ws/src/miracle_ai/miracle_ai/phm_predictor.py`
- New: `miracle_ws/src/miracle_twin/miracle_twin/cutting_sim_proxy.py`

**Implementation:**
- `CuttingSimProxy` accepts G-code blocks + current tool state + material properties
- Runs simplified Altintas force model (Python port of the Unity C# engine) to predict peak forces, power, and MRR per block
- Feeds predicted forces into Taylor wear model to estimate remaining tool life for the full program
- `prediction_runner.py` calls `CuttingSimProxy.simulate_program()` instead of returning templates
- Result: PHMPrediction with real RUL, confidence based on model fit, and trend data from the simulation
- Add `what_if` mode: user overrides spindle speed or feed, gets predicted wear/force comparison

**Tests:**
- `test_cutting_sim_proxy.py`: known G-code → verify force output within 10% of reference
- `test_prediction_runner_real.py`: end-to-end predict → verify non-hardcoded RUL

---

### 1.2 — G-Code Lookahead Engine

**Problem:** No ability to preview forces, collisions, or wear before cutting.

**Files:**
- New: `unity_twin/Assets/Scripts/Cutting/GCodeLookahead.cs`
- Modify: `unity_twin/Assets/Scripts/Cutting/GCodeExecutor.cs`
- Modify: `unity_twin/Assets/Scripts/UI/ForceChart.cs`

**Implementation:**
- `GCodeLookahead` takes the full parsed G-code program and current tool/material state
- Runs a fast-forward simulation (no voxel subtraction, just force/wear/thermal math) over the next N blocks (configurable, default 50)
- Produces a `LookaheadResult[]` with per-block predictions:
  - Peak force (N), power (W), temperature rise (°C)
  - Cumulative wear after block (mm), remaining tool life (min)
  - Chatter risk score (0-1) from stability lobe lookup
  - Collision flag (tool envelope vs. fixture bounding boxes)
- ForceChart gets a secondary "predicted" trace (dashed line) showing future force profile
- GCodeExecutor pauses and warns if any lookahead block exceeds force/wear/thermal thresholds
- Lookahead re-runs whenever the operator changes feed override or spindle override

**Collision detection:**
- Define fixture bounding boxes as ScriptableObject (`FixtureProfile`)
- Lookahead checks swept tool cylinder against fixture AABBs per block
- Visual: red highlight on toolpath segments with collision risk

**Tests:**
- `TestGCodeLookahead`: known pocket program → verify force predictions match within 15%
- `TestCollisionDetection`: tool path into fixture zone → flag raised

---

### 1.3 — Chatter Prediction (Stability Lobe Integration)

**Problem:** Chatter is detected from audio after it starts. The stability lobe chart exists but isn't used predictively.

**Files:**
- New: `unity_twin/Assets/Scripts/Cutting/StabilityLobePredictor.cs`
- Modify: `unity_twin/Assets/Scripts/Cutting/CuttingSimulationManager.cs`
- Modify: `unity_twin/Assets/Scripts/UI/StabilityLobeChart.cs`

**Implementation:**
- `StabilityLobePredictor` computes analytical stability boundary using:
  - Tool: diameter, flutes, helix angle, modal parameters (natural freq, damping ratio, stiffness)
  - Material: specific force coefficients (already in CuttingForceEngine)
  - Method: Altintas zeroth-order approximation (ZOA) for stability limit
- For each G-code block, plot (RPM, depth-of-cut) point on stability lobe diagram
- If point falls in unstable region → `ChatterRisk.HIGH`, recommend alternative RPM
- StabilityLobeChart renders the stability boundary curve + current operating point + lookahead points
- CuttingSimulationManager checks chatter risk before executing each block; warns if HIGH

**Modal parameters:** Default values for 1/4" HSS endmill in ER-11 collet (fn ≈ 1800 Hz, ζ ≈ 0.03, k ≈ 8e6 N/m). Expose as SerializeField for tap-test calibration.

**Tests:**
- Known unstable RPM/depth combination → risk HIGH
- Known stable combination → risk LOW
- Stability boundary matches reference curves within 20%

---

### 1.4 — DDS Encryption & SROS2 Enforcement

**Problem:** All ROS2 traffic is plaintext UDP. Any device on the network can sniff/spoof messages.

**Files:**
- Modify: `miracle_ws/docker/fastdds_profile.xml`
- Modify: `miracle_ws/scripts/generate_security.sh`
- Modify: `miracle_ws/docker/docker-compose.yaml`
- New: `miracle_ws/config/sros2_governance.xml`
- New: `miracle_ws/config/sros2_permissions.xml`

**Implementation:**
- Governance policy: encrypt all discovery + data traffic, sign all messages
- Permissions: per-node allow-lists (which topics each node can pub/sub)
- FastDDS profile: enable security plugins (`dds.sec.auth.builtin.PKI-DH`, `dds.sec.crypto.builtin.AES-GCM-GMAC`, `dds.sec.access.builtin.Governance`)
- `generate_security.sh` updated to:
  - Generate CA certificate + per-node certs (33 nodes)
  - Write governance.xml and permissions.xml
  - Sign permissions with CA key
- Docker compose: mount keystore volume, set `ROS_SECURITY_ENABLE=true`, `ROS_SECURITY_STRATEGY=Enforce`
- Environment variable `MIRACLE_SROS2_ENABLED=true` (default false for dev, true for production)

**Note:** Unity ROS-TCP-Connector uses TCP (not DDS), so the Unity↔bridge link needs separate TLS (see 1.5).

---

### 1.5 — G-Code Signing & Verification

**Problem:** G-code programs are loaded and executed without any integrity or provenance check. Malicious G-code could crash the tool into the workpiece.

**Files:**
- New: `miracle_ws/src/miracle_security/miracle_security/gcode_signer.py`
- Modify: `miracle_ws/src/miracle_cnc/miracle_cnc/gcode_executor.py` (ROS2 side)
- New: `unity_twin/Assets/Scripts/Cutting/GCodeSignatureVerifier.cs`
- Modify: `unity_twin/Assets/Scripts/Cutting/GCodeExecutor.cs` (Unity side)

**Implementation:**
- `gcode_signer.py`: CLI tool that signs `.nc` files
  - Computes SHA256 of file content
  - Signs hash with Ed25519 private key
  - Appends signature as comment block: `; MIRACLE_SIG:<base64_signature>`
  - Embeds signer ID and timestamp
- `GCodeSignatureVerifier.cs`: Unity-side verification
  - Strips signature comment, computes SHA256 of remaining content
  - Verifies Ed25519 signature against embedded public key
  - Returns `SignatureResult { Valid, Invalid, Missing }`
- GCodeExecutor (both sides): refuse to execute if signature is Invalid
  - Missing signature: warn but allow (configurable `requireSignedGCode` flag, default false)
- `ValidateGCode` service extended: check signature before bounds validation

**Key management:**
- Ed25519 key pair generated by `generate_security.sh`
- Public key embedded in Unity build as TextAsset
- Private key stays on the signing workstation (never in Docker/K8s)

---

### 1.6 — Wire Recovery Orchestrator

**Problem:** `recovery_orchestrator.py` logs "recovering..." but never actually restarts nodes.

**Files:**
- Modify: `miracle_ws/src/miracle_resiliency/miracle_resiliency/recovery_orchestrator.py`
- New: `miracle_ws/src/miracle_resiliency/miracle_resiliency/lifecycle_client.py`

**Implementation:**
- `LifecycleClient`: async wrapper around ROS2 lifecycle service calls
  - `change_state(node_name, transition)` → calls `/{node}/change_state`
  - Transitions: configure → activate → deactivate → cleanup → shutdown
  - Timeout per transition (default 10s)
- Recovery orchestrator strategies become real:
  - `IMMEDIATE_RESTART`: deactivate → cleanup → configure → activate
  - `RESTART_WITH_DEPENDENCIES`: topological sort dependency graph, restart in order
  - `DELAYED_RESTART`: schedule timer, then IMMEDIATE_RESTART
- Add attempt counter with max retries (default 3)
- On max retries exceeded: escalate to CRITICAL alert, stop retrying
- Add recovery success verification: after restart, wait for heartbeat within 2× timeout
- State machine: PENDING → IN_PROGRESS → VERIFYING → COMPLETED | FAILED

**Tests:**
- Mock lifecycle service → verify correct transition sequence called
- Max retries exceeded → verify escalation alert published
- Dependency restart → verify topological order respected

---

## Phase 2: Situational Awareness + Closed Loop

> Goal: Operators get correlated, predictive, actionable intelligence. The twin can feed back to the physical machine.

### 2.1 — Alert Correlation Engine

**Problem:** Every anomaly is an independent event. "Vibration spike on cnc1" + "thermal rise on cnc1" + "wear acceleration on cnc1" are three separate alerts when they're one causal chain.

**Files:**
- New: `miracle_ws/src/miracle_scada/miracle_scada/alert_correlator.py`
- New: `miracle_msgs/msg/CorrelatedAlert.msg`
- Modify: `miracle_ws/src/miracle_scada/miracle_scada/alarm_manager.py`

**Implementation:**
- `AlertCorrelator` ROS2 node subscribing to all anomaly + security + fleet health topics
- Correlation rules (configurable YAML):
  ```yaml
  rules:
    - name: tool_degradation_chain
      window_sec: 30
      conditions:
        - topic_pattern: "/miracle/+/anomaly"
          anomaly_type: CHATTER
        - topic_pattern: "/miracle/+/anomaly"
          anomaly_type: TOOL_WEAR
        - topic_pattern: "/miracle/+/anomaly"
          anomaly_type: THERMAL
      same_field: machine_id
      output:
        category: TOOL_DEGRADATION_CASCADE
        severity: CRITICAL
        recommendation: "Tool approaching end of life. Replace before next program."
  ```
- Sliding time window groups alerts by machine_id + time proximity
- Publishes `CorrelatedAlert` with:
  - `contributing_alerts[]` — IDs of constituent alerts
  - `root_cause_hypothesis` — most likely root cause from causal graph
  - `confidence` — based on rule match strength + temporal proximity
  - `recommended_actions[]` — prioritized list with cost/benefit estimates
- Alarm manager integrates: correlated alerts suppress individual constituents in the UI

**CorrelatedAlert.msg:**
```
builtin_interfaces/Time timestamp
string correlation_id
string category
string severity
string root_cause_hypothesis
float64 confidence
string[] contributing_alert_ids
string[] recommended_actions
string machine_id
```

---

### 2.2 — Operator Decision Support Panel

**Problem:** Recommendations are static strings ("Reduce feed rate"). No context, no cost/benefit, no alternatives.

**Files:**
- New: `unity_twin/Assets/Scripts/UI/DecisionSupportPanel.cs`
- Modify: `unity_twin/Assets/UI/Dashboard.uxml`
- Modify: `unity_twin/Assets/UI/Dashboard.uss`
- New: `unity_twin/Assets/ScriptableObjects/Events/CorrelatedAlertEventSO.cs`

**Implementation:**
- Right-side slide-out panel triggered by correlated alerts or operator request
- Panel sections:
  1. **Current Situation** — machine state + active anomalies + predictions in one view
  2. **Root Cause** — simplified causal chain visualization (A → B → C with confidence %)
  3. **Recommended Actions** — ranked list with:
     - Action description ("Replace tool T1")
     - Estimated downtime ("~5 min")
     - Risk if deferred ("Surface finish degrades to N7 within 12 min")
     - Cost impact ("Scrap risk increases 15% per hour")
  4. **What-If** — slider to simulate "what happens if I change feed to X?" using lookahead engine
- Actions are contextual:
  - Tool wear > 80% + rising vibration → "Replace tool" (primary), "Reduce feed 20%" (alternative)
  - Thermal anomaly + chatter → "Reduce RPM to next stable lobe" with specific RPM value
- Panel updates in real-time as conditions change

---

### 2.3 — Multi-Machine Fleet Comparison View

**Problem:** Only one machine visible at a time. No cross-machine pattern detection or benchmarking.

**Files:**
- New: `unity_twin/Assets/Scripts/UI/FleetOverviewPanel.cs`
- Modify: `unity_twin/Assets/UI/Dashboard.uxml`
- Modify: `unity_twin/Assets/UI/Dashboard.uss`

**Implementation:**
- Grid layout showing all machines simultaneously (2×2 or 3×1 depending on count)
- Per-machine card:
  - Status indicator (green/yellow/red)
  - Mini sparkline: spindle load last 60s
  - Tool wear bar
  - Current program + progress %
  - Active alert count (badge)
- Fleet-level metrics bar:
  - Average OEE across fleet
  - "Worst performer" highlight with reason
  - Shared resource status (coolant system, compressed air)
- Cross-machine anomaly correlation:
  - If same anomaly type appears on 2+ machines within 60s → highlight as "fleet-wide event"
  - Shared coolant temperature rise → flag as root cause candidate
- Click machine card → switches to detailed single-machine view

---

### 2.4 — Closed-Loop Adaptive Feedrate

**Problem:** The twin is read-only. It detects problems but can't prevent them.

**Files:**
- New: `miracle_ws/src/miracle_twin/miracle_twin/adaptive_controller.py`
- New: `miracle_msgs/msg/FeedOverride.msg`
- Modify: `miracle_ws/src/miracle_cnc/miracle_cnc/gcode_executor.py`
- Modify: `unity_twin/Assets/Scripts/Core/MiracleBridge.cs`

**Implementation:**
- `AdaptiveController` ROS2 node:
  - Subscribes to: MachineState, AnomalyAlert, PHMPrediction, lookahead results
  - Decision logic:
    - If predicted force > 80% of spindle capacity → reduce feed to stay at 70%
    - If chatter risk HIGH → shift RPM to nearest stable lobe pocket
    - If tool wear > 90% → reduce feed 50% and flag for tool change
    - If thermal > 85% of tool rating → reduce feed proportionally
  - Publishes `FeedOverride` with:
    - `feed_override_pct` (0-150%)
    - `spindle_override_pct` (0-120%)
    - `reason` (human-readable)
    - `confidence` (model confidence in the recommendation)
    - `revert_after_sec` (auto-revert timeout, 0 = manual revert)
  - **Safety constraint**: never increases feed beyond 100% of programmed value
  - **Operator override**: operator can dismiss any override via dashboard
- G-code executor applies override: `effective_feed = programmed_feed * override_pct / 100`
- MiracleBridge publishes override to Unity for visualization (feed bar turns yellow when overridden)
- All overrides logged to digital thread for traceability

**FeedOverride.msg:**
```
builtin_interfaces/Time timestamp
string machine_id
float64 feed_override_pct
float64 spindle_override_pct
string reason
float64 confidence
float64 revert_after_sec
```

---

### 2.5 — Explainable AI for Anomaly Alerts

**Problem:** Explanation generator returns "Detected CHATTER based on: audio_features". Not useful for operators.

**Files:**
- Modify: `miracle_ws/src/miracle_cognitive/miracle_cognitive/interface/explanation_generator.py`
- New: `miracle_msgs/msg/Explanation.msg`

**Implementation:**
- Structured explanations with three levels:
  1. **Summary** (1 line): "Chatter detected — vibration at 2.3 kHz exceeds stable cutting threshold"
  2. **Detail**: "The dominant frequency (2,317 Hz) aligns with the natural frequency of the tool assembly (2,280 Hz ± 5%). Energy ratio 3.4× baseline indicates resonant coupling between tool and workpiece."
  3. **Counterfactual**: "At 14,500 RPM (current: 16,000), the tooth-passing frequency shifts below the tool natural frequency, eliminating this chatter mode. Alternative: reduce axial depth from 0.8mm to 0.5mm."
- Feature contribution ranking: top 3 contributing signals with percentage contribution
- Confidence interval: "Model confidence: 87% (±6% based on training data coverage)"
- Historical context: "This pattern occurred 3 times in the last 7 days on this machine, always during pocket operations in 6061-T6"
- Published alongside every AnomalyAlert as a companion `Explanation` message

---

## Phase 3: Deep Security + Traceability

> Goal: Production-grade zero-trust security, full part genealogy, and validated resilience.

### 3.1 — Mutual TLS for ROS-TCP Bridge

**Problem:** Unity↔ROS bridge (port 10000) is plaintext TCP. Anyone on the network can inject commands.

**Files:**
- New: `miracle_ws/config/tls/` (CA cert, server cert/key, client cert/key)
- Modify: `miracle_ws/docker/docker-compose.yaml`
- Modify: `unity_twin/Assets/Scripts/Core/MiracleBridge.cs`

**Implementation:**
- Generate TLS certificates with `generate_security.sh`:
  - CA cert (self-signed, 10-year validity for lab use)
  - Server cert for ros_tcp_endpoint (CN=miracle-ros2)
  - Client cert for Unity (CN=unity-twin)
- ros_tcp_endpoint: wrap TCP socket with TLS (requires fork or proxy)
  - Alternative: stunnel sidecar container — `stunnel` terminates TLS on port 10001, forwards to ros_tcp_endpoint on port 10000 (localhost only)
  - Docker compose: add stunnel sidecar, expose only port 10001 externally
- Unity side: `SslStream` wrapper around `TcpClient` in ROSConnection
  - Load client cert from StreamingAssets
  - Validate server cert against embedded CA
- Fallback: if TLS handshake fails, log warning and connect plaintext (configurable `requireTLS` flag)

---

### 3.2 — Encrypted & Signed Audit Logs

**Problem:** Audit logs in `/tmp/miracle_audit/` are plaintext, world-readable, deletable.

**Files:**
- Modify: `miracle_ws/src/miracle_security/miracle_security/audit_logger.py`
- New: `miracle_ws/src/miracle_security/miracle_security/secure_storage.py`

**Implementation:**
- `SecureStorage` class:
  - AES-256-GCM encryption for log entries (key from environment variable or K8s secret)
  - Each entry: `[nonce(12)][ciphertext][tag(16)]` in binary format
  - Ed25519 signature over the hash chain (not just SHA256 linking)
  - Signature covers: `sign(prev_hash || entry_hash || sequence_number)`
- Storage location: configurable, default `/var/miracle/audit/` (not /tmp)
  - K8s: PersistentVolumeClaim with ReadWriteOnce
  - Docker: named volume with restricted permissions
- Log rotation: keep encrypted archives, compress with gzip
- Integrity verification: `verify_chain()` validates all signatures + hash links
- Access control: file permissions 0600, owned by miracle user

---

### 3.3 — Material Genealogy in Digital Thread

**Problem:** No traceability from raw material batch → tool usage → finished part serial number.

**Files:**
- Modify: `miracle_ws/src/miracle_mes/miracle_mes/digital_thread.py`
- New: `miracle_msgs/msg/MaterialBatch.msg`
- New: `miracle_msgs/msg/PartRecord.msg`
- Modify: `miracle_ws/src/miracle_mes/miracle_mes/job_scheduler.py`

**Implementation:**
- Extend digital thread entry types:
  - `MATERIAL_LOADED`: batch ID, material type, supplier, dimensions, cert reference
  - `TOOL_INSTALLED`: tool ID, type, initial wear state, max life
  - `OPERATION_COMPLETE`: program name, actual cycle time, material removed (mm³), final wear
  - `PART_COMPLETE`: serial number, final dimensions (if measured), pass/fail, linked entries
  - `TOOL_REMOVED`: final wear state, total cutting time, total material removed
- Each entry links to previous via SHA256 chain (already exists)
- New queries:
  - `get_part_history(serial)` → full chain from material batch through all operations
  - `get_tool_history(tool_id)` → all parts cut, total wear, materials processed
  - `get_batch_traceability(batch_id)` → all parts produced from this material batch
- Job scheduler publishes MATERIAL_LOADED when job starts, PART_COMPLETE when job ends
- Digital thread exposes query service for downstream MES/ERP integration

**MaterialBatch.msg:**
```
builtin_interfaces/Time timestamp
string batch_id
string material_type
string supplier
float64[3] dimensions_mm
string certification_reference
```

**PartRecord.msg:**
```
builtin_interfaces/Time timestamp
string serial_number
string job_id
string machine_id
string material_batch_id
string[] tool_ids_used
float64 cycle_time_sec
float64 material_removed_mm3
string quality_result
string[] digital_thread_entry_ids
```

---

### 3.4 — Real Firmware Attestation

**Problem:** `attestation_verifier.py` computes `SHA256("device_id:challenge")` — completely simulated.

**Files:**
- Modify: `miracle_ws/src/miracle_security/miracle_security/attestation_verifier.py`
- New: `miracle_ws/src/miracle_security/miracle_security/tpm_interface.py`

**Implementation:**
- Two-tier approach:
  1. **Tier 1 (MCU devices):** Arduino/nRF52840 compute SHA256 of their own flash contents and report
     - Firmware hash: SHA256 of application binary region
     - Config hash: SHA256 of EEPROM/NVS configuration
     - Challenge-response: device computes HMAC-SHA256(shared_secret, challenge || firmware_hash)
     - Shared secret provisioned at device setup (stored in nRF52840 UICR protected region)
  2. **Tier 2 (Linux nodes):** Use TPM 2.0 or software TPM (swtpm for dev)
     - `tpm_interface.py`: wraps `tpm2-tools` CLI (`tpm2_quote`, `tpm2_verifysignature`)
     - PCR-based attestation: read PCR banks 0-7 (firmware, bootloader, OS, app)
     - Quote: TPM signs PCR values with AIK (Attestation Identity Key)
     - Verifier checks quote signature against enrolled AIK public key
- Attestation verifier updated:
  - Store known-good hashes per device (from provisioning step)
  - Compare reported hashes against known-good
  - Trust score: 1.0 if match, 0.0 if mismatch, decay if overdue
  - Quarantine: trust < 0.3 → publish SecurityAlert + IsolateNode

---

### 3.5 — Chaos Engineering Execution

**Problem:** `chaos_injector.py` tracks faults but never actually injects them.

**Files:**
- Modify: `miracle_ws/src/miracle_resiliency/miracle_resiliency/chaos_injector.py`
- New: `miracle_ws/src/miracle_resiliency/miracle_resiliency/fault_executor.py`

**Implementation:**
- `FaultExecutor` class with pluggable fault types:
  1. **Network delay**: inject latency via `tc qdisc` (Linux traffic control) on container network interface
     - `tc qdisc add dev eth0 root netem delay {ms}ms`
     - Cleanup: `tc qdisc del dev eth0 root`
  2. **Node kill**: send lifecycle transition `shutdown` to target node
     - Cleanup: restart via recovery orchestrator
  3. **CPU stress**: spawn `stress-ng --cpu N` process in target container
     - Cleanup: kill stress-ng process
  4. **Memory pressure**: `stress-ng --vm 1 --vm-bytes {MB}M`
     - Cleanup: kill stress-ng process
  5. **Message drop**: subscribe to target topic, republish with configurable drop rate
     - Cleanup: remove intermediary subscription
- Safety controls:
  - `enabled` flag must be explicitly true (default false)
  - Max concurrent faults: 3
  - Max fault duration: 60s (hard cap)
  - Excluded nodes: heartbeat_aggregator, recovery_orchestrator, chaos_injector itself
  - Auto-cleanup on node shutdown (destructor removes all active faults)
- Fault results published: `FaultResult` with actual impact metrics (latency measured, messages dropped, recovery time)

---

## Execution Order

```
Phase 1 (parallel where possible):
  1.1 + 1.2 + 1.3  (prediction + lookahead + chatter — all Unity/Python, independent)
  1.4 + 1.5         (SROS2 + G-code signing — both security, independent)
  1.6               (recovery orchestrator — depends on nothing)

Phase 2 (after Phase 1, parallel where possible):
  2.1 + 2.3         (alert correlation + fleet view — independent)
  2.2               (decision support — depends on 2.1 for correlated alerts)
  2.4               (closed-loop — depends on 1.2 lookahead results)
  2.5               (explainable AI — depends on 2.1 correlation context)

Phase 3 (after Phase 2, parallel where possible):
  3.1 + 3.2 + 3.4   (mTLS + encrypted logs + attestation — independent)
  3.3               (material genealogy — independent)
  3.5               (chaos engineering — depends on 1.6 recovery orchestrator)
```

## New Files Summary

| File | Domain |
|------|--------|
| `miracle_ws/src/miracle_twin/miracle_twin/cutting_sim_proxy.py` | Digital Twin |
| `unity_twin/Assets/Scripts/Cutting/GCodeLookahead.cs` | Digital Twin |
| `unity_twin/Assets/Scripts/Cutting/StabilityLobePredictor.cs` | Digital Twin |
| `miracle_ws/config/sros2_governance.xml` | Security |
| `miracle_ws/config/sros2_permissions.xml` | Security |
| `miracle_ws/src/miracle_security/miracle_security/gcode_signer.py` | Security |
| `unity_twin/Assets/Scripts/Cutting/GCodeSignatureVerifier.cs` | Security |
| `miracle_ws/src/miracle_resiliency/miracle_resiliency/lifecycle_client.py` | Resiliency |
| `miracle_ws/src/miracle_scada/miracle_scada/alert_correlator.py` | Situational Awareness |
| `miracle_msgs/msg/CorrelatedAlert.msg` | Situational Awareness |
| `unity_twin/Assets/Scripts/UI/DecisionSupportPanel.cs` | Situational Awareness |
| `unity_twin/Assets/Scripts/UI/FleetOverviewPanel.cs` | Situational Awareness |
| `miracle_ws/src/miracle_twin/miracle_twin/adaptive_controller.py` | Closed Loop |
| `miracle_msgs/msg/FeedOverride.msg` | Closed Loop |
| `miracle_msgs/msg/Explanation.msg` | Situational Awareness |
| `miracle_ws/src/miracle_security/miracle_security/secure_storage.py` | Security |
| `miracle_msgs/msg/MaterialBatch.msg` | Digital Twin |
| `miracle_msgs/msg/PartRecord.msg` | Digital Twin |
| `miracle_ws/src/miracle_security/miracle_security/tpm_interface.py` | Security |
| `miracle_ws/src/miracle_resiliency/miracle_resiliency/fault_executor.py` | Resiliency |
| `unity_twin/Assets/ScriptableObjects/Events/CorrelatedAlertEventSO.cs` | Situational Awareness |

## Modified Files Summary

| File | Change |
|------|--------|
| `prediction_runner.py` | Replace hardcoded outputs with real simulation |
| `phm_predictor.py` | Accept lookahead data for improved RUL |
| `GCodeExecutor.cs` | Integrate lookahead warnings + signature check |
| `ForceChart.cs` | Add predicted force trace (dashed) |
| `CuttingSimulationManager.cs` | Check chatter risk per block |
| `StabilityLobeChart.cs` | Render stability boundary + operating point |
| `fastdds_profile.xml` | Enable SROS2 security plugins |
| `generate_security.sh` | Add TLS certs, governance/permissions |
| `docker-compose.yaml` | SROS2 env vars, stunnel sidecar, keystore volume |
| `gcode_executor.py` (ROS2) | Signature verification + feed override |
| `MiracleBridge.cs` | Publish/subscribe FeedOverride + TLS support |
| `recovery_orchestrator.py` | Real lifecycle transitions + retry logic |
| `alarm_manager.py` | Integrate correlated alerts |
| `explanation_generator.py` | Structured 3-level explanations |
| `Dashboard.uxml` | Decision support panel + fleet overview |
| `Dashboard.uss` | Styles for new panels |
| `digital_thread.py` | Material/part entry types + queries |
| `job_scheduler.py` | Publish material/part lifecycle events |
| `attestation_verifier.py` | Real firmware hash verification |
| `audit_logger.py` | AES-256-GCM encryption + Ed25519 signatures |
| `chaos_injector.py` | Wire to fault executor |
