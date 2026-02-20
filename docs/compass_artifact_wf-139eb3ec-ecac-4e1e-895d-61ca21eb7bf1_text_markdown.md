# Building a digital twin for an automated manufacturing cell in Unity

**A voxel-based cutting simulation driven by 12 wireless sensors, a collaborative robot, and a desktop CNC—all unified through MQTT—is entirely feasible with the architecture described below.** The critical constraint is BLE: the ArduinoBLE library caps notification throughput at ~40 Hz per device, and Windows supports only ~7 concurrent BLE connections, making a dedicated nRF52840 gateway essential for your 12-sensor network. For material removal, voxel grids with GPU marching cubes outperform mesh-based CSG, which degrades catastrophically after repeated boolean operations. The Niryo Ned2 integrates cleanly via a lightweight PyNiryo2-to-MQTT Python bridge, keeping your entire data plane on a single protocol. This guide covers every subsystem in detail—from Arduino sketch patterns to compute shader strategies for real-time SDF subtraction.

---

## Part 1: BLE sensor network architecture

### Arduino Nano 33 BLE Sense as peripheral

The Nano 33 BLE Sense boards (nRF52840 SoC, Bluetooth 5.0) act as BLE peripherals using the **ArduinoBLE library v1.3.x**. Define custom 128-bit UUID services for each data type. The most efficient pattern packs all 9 IMU axes into a single 36-byte `BLECharacteristic` with `BLERead | BLENotify` properties, rather than creating 9 separate float characteristics:

```cpp
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>    // Rev1; use Arduino_BMI270_BMM150 for Rev2
#include <Arduino_HTS221.h>

BLEService imuService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic imuChar("19B10011-E8F2-537E-4F6C-D104768A1214",
                          BLERead | BLENotify, 36, true);  // 9 floats packed

BLEService envService("19B10020-E8F2-537E-4F6C-D104768A1214");
BLEFloatCharacteristic tempChar("2A6E", BLERead | BLENotify);
BLEFloatCharacteristic humChar("2A6F", BLERead | BLENotify);
```

In the main loop, pack raw floats into a byte buffer and call `imuChar.writeValue(buf, 36)` to trigger a notification. Use `BLE.setConnectionInterval(8, 16)` (10–20 ms) for faster data delivery. Environment data (temperature, humidity) should stream at a slower cadence—**1 Hz is sufficient**—to conserve bandwidth for IMU data.

**The critical limitation**: ArduinoBLE caps effective notification throughput at roughly **40 Hz** regardless of connection interval settings (a known firmware-level constraint documented in GitHub issue #68). To achieve your target of 100 Hz IMU sampling, either batch 2–3 samples per notification at 33–50 Hz, or encode values as `int16` instead of `float` (18 bytes per sample instead of 36, fitting within a single 20-byte BLE packet). The int16 approach halves per-device bandwidth from 3,600 bytes/sec to 1,800 bytes/sec with negligible precision loss for vibration analysis.

### One central cannot handle 12 connections on Windows

The Windows WinRT BLE stack supports a practical maximum of **5–7 simultaneous connections**, confirmed by Microsoft documentation and extensive community testing. Linux BlueZ fares better at 7–10 per adapter but still falls short of 12 with reliable high-throughput notifications. **Do not plan on using a Windows PC directly as the BLE central for 12 devices.**

The recommended gateway is an **nRF52840 Development Kit or USB Dongle** (~$10–40). Nordic's S140 SoftDevice supports **up to 20 concurrent BLE connections** as a central, with proven multi-link examples (Nordic's `nrf52-ble-multi-link-multi-role` demo connects 19 peripherals simultaneously). The nRF52840 aggregates all sensor data and forwards it to the PC via USB serial (CDC-ACM), completely bypassing the OS Bluetooth stack. Use the nRF5 SDK 17.0.2 with SoftDevice v7.2.0 rather than the Zephyr-based nRF Connect SDK, which has reported stability issues above 6 connections.

An alternative is a **Raspberry Pi 4 with two USB Bluetooth 5.0 adapters** (6 devices per adapter), running `bleak` on Linux. Each adapter operates as an independent HCI controller (`hci0`, `hci1`), and bleak's `adapter` parameter lets you assign devices to specific adapters.

### Throughput math confirms feasibility

With 12 connections on an nRF52840 gateway using BLE 5.0 Data Length Extension, each connection event delivers up to **244 bytes** of ATT payload. At a conservative 30 ms connection interval, each device achieves ~8,133 bytes/sec—well above the 3,600 bytes/sec required for 100 Hz IMU at 36 bytes/sample (or 1,800 bytes/sec with int16 encoding). Total aggregate throughput of **43.2 KB/sec across 12 devices** is comfortably within BLE 5.0 capacity.

### BLE-to-MQTT bridge using bleak and paho-mqtt

The bridge runs as a single Python process using **bleak v2.1+** (asyncio-based BLE client, MIT license) and **paho-mqtt v2.x**. All 12 `BleakClient` connections share one asyncio event loop via `asyncio.gather()`. The paho-mqtt client runs its own background thread via `loop_start()`, coexisting with asyncio. The critical pattern: scan once with `BleakScanner.discover()`, then pass the discovered `BLEDevice` objects (not raw addresses) to `BleakClient` to avoid implicit re-scanning conflicts:

```python
import asyncio, struct, json
from bleak import BleakClient, BleakScanner
import paho.mqtt.client as mqtt

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.connect("localhost", 1883)
mqttc.loop_start()

def make_handler(device_id):
    def handler(sender, data):
        values = struct.unpack('<9f', data)
        mqttc.publish(f"dt/sensors/{device_id}/imu",
                      json.dumps({"accel": list(values[0:3]),
                                  "gyro": list(values[3:6]),
                                  "mag": list(values[6:9])}), qos=0)
    return handler

async def connect_device(ble_device, device_id):
    while True:
        try:
            async with BleakClient(ble_device, timeout=30) as client:
                await client.start_notify(IMU_CHAR_UUID, make_handler(device_id))
                while client.is_connected:
                    await asyncio.sleep(1.0)
        except Exception:
            await asyncio.sleep(5)  # reconnect with backoff

async def main():
    devices = await BleakScanner.discover(timeout=10.0)
    targets = {d.address: d for d in devices if d.address in KNOWN_ADDRESSES}
    await asyncio.gather(*[connect_device(dev, addr) for addr, dev in targets.items()])
```

Use **QoS 0** for high-frequency IMU data (lost samples are acceptable) and **QoS 1** for configuration commands. If using the nRF52840 gateway approach instead, the bridge simplifies to serial port parsing—read aggregated binary frames from USB serial and publish to MQTT topics.

---

## Part 2: Niryo Ned2 robot integration

### Communication options and the MQTT recommendation

The Ned2 runs **ROS 1 Noetic on Ubuntu 20.04** (Raspberry Pi 4B) and exposes four communication interfaces: PyNiryo/PyNiryo2 Python API, native ROS topics/services, TCP/IP, and Modbus TCP (port 5020). **PyNiryo2 v1.0.0** is the newer API, built on `roslibpy` (WebSocket/ROSBridge), supporting callbacks, async operations, and parallelism.

**The recommended integration is a lightweight PyNiryo2-to-MQTT bridge** rather than native ROS. This keeps the entire system on a single protocol (MQTT) with no ROS installation required on the Unity host. The bridge script polls joint states at 10–30 Hz and publishes to MQTT:

```python
from pyniryo2 import NiryoRobot
import paho.mqtt.client as mqtt
import json, time

robot = NiryoRobot("192.168.1.x")
mqttc = mqtt.Client()
mqttc.connect("mqtt_broker_ip", 1883)

def joint_state_callback(joint_state):
    mqttc.publish("dt/robot/joint_states", json.dumps({
        "joints": list(joint_state.position),
        "timestamp": time.time()
    }))

robot.arm.joint_states_callbacks.append(joint_state_callback)
```

The Ned2 provides rich data for digital twin visualization: **6 joint positions** (radians, ~50 Hz via ROS, 10–30 Hz via API), joint velocities and efforts, TCP pose (x/y/z/roll/pitch/yaw), gripper state (open/close/torque), hardware status (per-motor temperatures, voltages, error codes), LED ring color/mode, and robot state (BOOTING, CALIBRATING, IDLE, MOVING, PAUSE, ERROR). Modbus TCP provides a simpler but more limited interface—joint positions at addresses 0–5 as milliradians.

If deeper ROS integration is later needed (full TF tree, MoveIt planning), the Ned2 already runs a ROSBridge server on port 9090, and **ROS# (ros-sharp) by Siemens** can connect directly via WebSocket without additional robot-side setup. Niryo also provides a `ned-ros2-driver` that bridges ROS1 on the robot to ROS2 on a remote PC.

### URDF import and robot animation in Unity

The Ned2 URDF files live in the `niryo_robot_description` package within the [ned_ros GitHub repository](https://github.com/NiryoRobotics/ned_ros). Convert XACRO to plain URDF first (`rosrun xacro xacro --inorder -o ned2.urdf ned2.urdf.xacro`), then import using Unity's **URDF Importer package** (`com.unity.robotics.urdf-importer` v0.5.2). The importer creates a GameObject hierarchy with **ArticulationBody** components on each link, automatically mapping revolute joints with limits from the URDF. Set Physics → Solver Type to **Temporal Gauss Seidel** to prevent erratic joint behavior.

The resulting hierarchy is: `base_link → shoulder_link (joint_1) → arm_link (joint_2) → elbow_link (joint_3) → forearm_link (joint_4) → wrist_link (joint_5) → hand_link (joint_6) → tool_link → gripper`. For a visualization-only digital twin, drive joints via direct Transform rotation (`Quaternion.AngleAxis`) rather than physics-based ArticulationBody drives—it's simpler and sufficient for mirroring the physical robot:

```csharp
void ApplyJointAngles(float[] jointsRadians) {
    for (int i = 0; i < 6; i++) {
        float deg = jointsRadians[i] * Mathf.Rad2Deg;
        jointTransforms[i].localRotation = 
            Quaternion.AngleAxis(deg, jointAxes[i]);
    }
}
```

Interpolate between received joint state frames using `Mathf.Lerp` (or `Quaternion.Slerp` for angles) to achieve smooth 60 fps rendering from 10–30 Hz MQTT updates. For the gripper, the Ned2 has two symmetric finger joints (mors_1, mors_2)—animate them symmetrically based on the gripper open/close state received via MQTT.

Note three URDF import caveats: `package://` mesh paths must be converted to relative paths; XACRO macros are not processed by Unity; and mimic joints (used by the gripper fingers) need custom scripting since ArticulationBody doesn't natively support them.

---

## Part 3: Material removal via voxels, not CSG

### Why mesh-based CSG fails for machining simulation

Mesh-based boolean operations (BSP-tree approach, as implemented by pb_CSG, Net3dBool, and similar Unity packages) suffer from **catastrophic degradation after repeated operations**. Each subtraction increases polygon count by O(n), and floating-point errors accumulate rapidly. After 100+ subtractions—routine in a machining simulation—meshes develop holes, degenerate triangles, and unmanageable polygon counts (tens of thousands). Multiple Unity forum threads confirm this: "mesh gets destroyed" after sequential boolean ops. Unity's ProBuilder CSG is experimental and editor-only. Realtime CSG (Sabresaurus) bakes all operations at build time. **No mesh-based CSG solution is viable for continuous CNC simulation.**

### Voxel grid with GPU marching cubes is the recommended approach

Represent the workpiece as a 3D grid of binary voxels (1 = material present, 0 = removed). Material removal sets voxels along the toolpath to 0. Surface visualization uses Marching Cubes to extract a triangle mesh from the voxel field. This approach has **zero degradation over millions of operations**, constant-time per subtraction (only affected voxels are tested), and is embarrassingly parallel on GPU.

For the Bantam Tools work volume (178 × 229 × 83 mm), a **256³ voxel grid** provides approximately **0.7–0.9 mm resolution** and consumes only ~2 MB of memory (1 bit per voxel). A 512³ grid achieves ~0.35–0.45 mm resolution at 16 MB. For finer detail, use a **Sparse Voxel Octree (SVO)** that subdivides only near the cutting zone—at depth 12, resolution reaches ~43 μm while keeping memory manageable.

The GPU pipeline for material removal:

1. Store the voxel grid in a `RWStructuredBuffer<uint>` (compute buffer) on the GPU
2. For each toolpath segment, compute the bounding box of the swept tool volume
3. Dispatch a compute shader that tests each voxel against the tool cylinder and sets affected bits to 0
4. Run Marching Cubes on modified chunks only (use dirty-flag tracking per chunk)
5. Render the resulting mesh via `DrawProceduralIndirect` to keep all data GPU-side

```hlsl
// Compute shader: subtract tool from voxel grid
[numthreads(8, 8, 8)]
void CSSubtractTool(uint3 id : SV_DispatchThreadID) {
    float3 worldPos = VoxelToWorld(id);
    float distToToolAxis = length(worldPos.xz - toolPos.xz);
    if (distToToolAxis < toolRadius && 
        worldPos.y > toolTip.y && worldPos.y < toolTip.y + toolLength) {
        VoxelGrid[FlatIndex(id)] = 0;
    }
}
```

Keijiro Takahashi's **ComputeMarchingCubes** (github.com/keijiro/ComputeMarchingCubes) provides a production-quality GPU marching cubes implementation for Unity. For CPU-side alternatives, the Javier-Garzo Marching Cubes implementation supports Job System + Burst compilation with chunk-based updates.

### SDF approach for higher fidelity

For sub-millimeter accuracy, store a **Signed Distance Field** in a 3D `RWTexture3D<float>`. Boolean subtraction becomes a trivially parallel GPU operation: `sdf_result = max(workpiece_sdf, -tool_sdf)`. Mesh extraction via Marching Cubes samples the SDF at the zero-crossing isosurface. The **IsoMesh project** (github.com/EmmetOT/IsoMesh) provides a complete Unity pipeline: mesh → SDF → manipulation → mesh extraction via surface nets, all in compute shaders. Composite Adaptively Sampled Distance Fields (research from Mitsubishi Electric Research Labs) achieved **sub-micron accuracy** for CNC simulation with low memory requirements. Consider SDF as a Phase 2 upgrade after proving the voxel approach.

### G-code parsing to mesh modification

Parse G-code using **gsGCode** (github.com/gradientspace/gsGCode, MIT license, C#) or a custom regex-based parser. Extract G0 (rapid), G1 (linear cut), G2/G3 (arc) commands with X/Y/Z/I/J/K/F parameters. Convert toolpath segments to swept volumes by discretizing the path into small steps (~0.1 mm), placing the tool cylinder at each step, and subtracting from the voxel grid. For arcs (G2/G3), discretize into linear segments at 1–5° angular increments. This "stamping" approach is simpler than exact swept-volume computation and works naturally with the voxel representation.

---

## Part 4: Chip formation physics and cutting force visualization

### Cutting parameters for the Bantam with a 1/4" HSS end mill

The Bantam Tools Desktop CNC has a **250 W spindle** (ER-11 collet, 10,000–28,000 RPM range) and a work volume of 178 × 229 × 83 mm. For a 1/4" (6.35 mm) 2-flute HSS end mill in 6061-T6 aluminum with dry cutting:

| Parameter | Roughing | Finishing |
|-----------|----------|-----------|
| Spindle speed | 12,000–16,000 RPM | 16,000–20,000 RPM |
| Feed rate | 760–1,270 mm/min | 380–760 mm/min |
| Axial depth of cut | 0.5–1.0 mm | 0.13–0.38 mm |
| Radial depth of cut | 30–50% of D | 5–15% of D |
| Feed per tooth | 0.025–0.076 mm | 0.013–0.025 mm |

Bantam's machine works best with **high speed and shallow cuts** due to the low-torque spindle. At 12,000 RPM with a 6.35 mm end mill, surface speed reaches ~785 SFM—high for HSS, but acceptable with light cuts and good chip evacuation in dry milling.

### Mechanistic cutting force model

The instantaneous undeformed chip thickness varies sinusoidally with rotation angle: **h(φ) = fz × sin(φ)**, where fz is feed per tooth. The specific cutting force for aluminum follows the Kienzle model: **Kc = Kc1 × h^(−MC)**, where Kc1 ≈ **600–700 N/mm²** for Al 6061-T6 and MC ≈ 0.25. At a chip thickness of 0.05 mm, this yields Kc ≈ 1,480 N/mm².

Differential cutting forces per tooth element:

- Tangential: dFt = Ktc × h(φ) × dz + Kte × dz
- Radial: dFr = Krc × h(φ) × dz + Kre × dz  
- Axial: dFa = Kac × h(φ) × dz + Kae × dz

For the desktop CNC at typical parameters, expect **peak resultant forces of 10–40 N** and cutting power of approximately 10–25 W (within the 250 W spindle capacity). The voxel-based material removal model naturally supports force calculation: count removed voxels per time step to compute material removal rate, then multiply by specific cutting pressure.

### Chip visualization with VFX Graph

Aluminum generates **primarily continuous chips** (long curled strips) at the speeds used on desktop CNCs. Built-up edge formation is a concern at intermediate speeds with HSS tooling.

Use Unity's **VFX Graph** (GPU-based particle system) for chip visualization—it handles millions of particles versus thousands for the legacy Particle System. Pre-model 3–5 curled aluminum chip mesh variations, then configure VFX Graph with mesh rendering mode. Emit particles at the cutter-workpiece engagement zone with initial velocity tangential to tool rotation plus a feed-direction component. Apply curl via Force over Lifetime and random size/rotation variation. VFX Graph runs entirely on GPU and integrates with both URP and HDRP.

For cutting force visualization, render arrow meshes via `Graphics.DrawMeshInstancedIndirect` with per-instance direction/magnitude from a ComputeBuffer (single draw call for all force vectors). Color-code by component: red = tangential, green = radial, blue = axial. Heat maps on the workpiece surface use a custom Shader Graph node that samples a temperature data texture (uploaded as a ComputeBuffer) and maps values to a blue-to-red color ramp. For real-time plotting of force, vibration, and temperature data, **Graph And Chart** (BitSplash Interactive, Unity Asset Store) supports streaming line graphs with real-time data updates.

---

## Part 5: Three timing modes with a custom simulation clock

### Why Time.timeScale is insufficient

Unity's `Time.timeScale` is global (affects all systems simultaneously), capped at 100 in the Editor, cannot go negative for rewind, and disrupts physics globally. A manufacturing digital twin needs independent control: the simulation running at 10× while UI animations remain at 1×, or scrubbing through recorded data while the 3D viewport updates interactively. **A custom `SimulationClock` is essential.**

```csharp
public class SimulationClock : MonoBehaviour {
    public enum Mode { RealTime, Accelerated, Replay, Paused }
    public Mode CurrentMode { get; private set; }
    public double SimulationTime { get; private set; }
    public double DeltaTime { get; private set; }
    public float SpeedMultiplier { get; set; } = 1.0f;
    public event Action<double> OnTimeAdvanced;

    void Update() {
        switch (CurrentMode) {
            case Mode.RealTime:
                DeltaTime = Time.unscaledDeltaTime;
                break;
            case Mode.Accelerated:
                DeltaTime = Time.unscaledDeltaTime * SpeedMultiplier;
                break;
            case Mode.Replay:
                return; // Driven externally by ReplayController
            case Mode.Paused:
                DeltaTime = 0; break;
        }
        SimulationTime += DeltaTime;
        OnTimeAdvanced?.Invoke(SimulationTime);
    }
}
```

All subsystems (CNC manager, robot manager, sensor manager, cutting simulation) subscribe to `OnTimeAdvanced` rather than reading `Time.deltaTime` directly.

### Real-time sync mode

Receive live CNC position data via MQTT and update the simulation in lockstep. Use **buffered interpolation**: intentionally delay rendering by 100–150 ms (for CNC with ~50 ms network latency) and store timestamped position snapshots in a ring buffer. Each frame, find the two snapshots bracketing the delayed render time and linearly interpolate between them. This absorbs jitter and ensures smooth visualization:

```csharp
float renderTime = (float)(SimClock.SimulationTime - interpolationDelay);
var (before, after) = ringBuffer.GetBracketing(renderTime);
float t = (renderTime - before.time) / (after.time - before.time);
toolPosition = Vector3.Lerp(before.position, after.position, t);
```

Use `ConcurrentQueue<T>` to transfer MQTT data from the background callback thread to the Unity main thread, processing a capped number of messages per frame to avoid spikes.

### Accelerated preview mode

Parse the entire G-code file into a `List<ToolpathSegment>` with pre-computed positions, motion types, feed rates, and durations. Each frame, advance simulation time by `unscaledDeltaTime × speedMultiplier` and process all segments that fall within that window. The cutting simulation (voxel subtraction + marching cubes) runs in a coroutine or Job System pipeline, consuming segments as fast as the GPU can process material removal. Decouple the simulation state machine from Unity's render loop—the visual representation samples the current simulation state each frame.

### Post-process replay mode

Log all data streams during live operation as timestamped records in JSON or binary (MessagePack for large recordings). Each stream (CNC position at 20 Hz, robot joints at 50 Hz, sensors at 5–40 Hz) maintains its own sorted `List<TimestampedSample<T>>`. The `ReplayController` drives a single playback clock with play, pause, rewind, fast-forward, and scrub-to-timestamp controls. Each stream independently binary-searches for bracketing samples and interpolates against the same clock, naturally handling different sample rates.

---

## Part 6: Unified system architecture on MQTT

### Topic hierarchy

Follow ISA-95/Unified Namespace principles with a clean hierarchy:

```
dt/
├── cnc/
│   ├── position          # {x, y, z} — QoS 0, ~20 Hz
│   ├── status            # {state, mode, error} — QoS 1, retained
│   ├── spindle           # {rpm, load} — QoS 0
│   ├── feed              # {rate, override_pct} — QoS 0
│   └── gcode/current     # {line_num, block} — QoS 0
├── robot/
│   ├── joint_states      # {j1..j6, velocities} — QoS 0, ~30 Hz
│   ├── gripper           # {state, torque} — QoS 0
│   ├── status            # {mode, error} — QoS 1, retained
│   └── tcp_pose          # {x, y, z, rx, ry, rz} — QoS 0
├── sensors/
│   ├── {01..12}/
│   │   ├── imu           # {accel, gyro, mag} — QoS 0, ~40 Hz
│   │   ├── audio         # {rms, peak_freq} — QoS 0
│   │   └── environment   # {temperature, humidity} — QoS 0, ~1 Hz
│   └── _meta/config      # sensor positions — QoS 1, retained
└── sim/
    ├── mode              # {realtime|accelerated|replay} — QoS 1, retained
    ├── time              # {sim_time, speed} — QoS 0
    └── cmd/set_mode      # QoS 1
```

Use **QoS 0** for high-frequency telemetry (position, IMU, audio) where occasional sample loss is acceptable. Use **QoS 1** for commands and status changes. Use **retained messages** on all `*/status` topics so new subscribers immediately get current state.

### Event-driven decoupling with ScriptableObject channels

Unity Technologies officially recommends **ScriptableObject-based event channels** for decoupled communication between subsystems. Create typed SO assets (`CNCPositionEventSO`, `RobotJointStateEventSO`, `SensorDataEventSO`, `SimModeChangedEventSO`) that act as project-level relay points. The MQTT service parses incoming topics and raises events on the appropriate channel; subsystem managers register as listeners. No manager holds a direct reference to any other:

```
MQTTService → parses topics → raises SO events
    → CNCManager (updates tool position, spindle visualization)
    → RobotManager (updates joint angles, gripper state)
    → SensorManager (updates 12 sensor displays, runs analysis)
    → CuttingSimulation (drives material removal)
    → RecordingService (logs to file for replay)
    → UIManager (updates dashboard)
```

### MQTT client in Unity

Use **M2MqttUnity** (github.com/gpvigano/M2MqttUnity) as the MQTT client. It wraps the M2Mqtt library with a MonoBehaviour base class that handles thread marshaling: MQTT callbacks queue messages on a background thread, and `Update()` processes them on the Unity main thread. Subclass `M2MqttUnityClient`, override `DecodeMessage()`, and dispatch to the appropriate ScriptableObject event channel. For MQTT v5 features, MQTTnet is an alternative but requires manual DLL import and thread marshaling.

### Synchronization across sources with different latencies

| Source | Update rate | Latency | Interpolation delay |
|--------|------------|---------|---------------------|
| CNC (TinyG → MQTT) | ~20 Hz | ~50 ms | 100 ms |
| BLE sensors | ~5–40 Hz | ~100–200 ms | 300 ms |
| Niryo Ned2 | ~30 Hz | ~20–40 ms | 60 ms |

Each source maintains its own `TimestampedRingBuffer` (pre-allocated, capacity ~60 samples). All devices stamp messages with NTP-synchronized clocks—standard NTP on a LAN achieves **~5–10 ms accuracy**, sufficient since BLE jitter (±50 ms) is the limiting factor. The rendering pipeline subtracts each source's interpolation delay from the simulation clock and interpolates independently.

---

## Part 7: Performance optimization for the full workload

### Frame budget at 30 FPS (33.3 ms per frame)

| Subsystem | Budget | Approach |
|-----------|--------|----------|
| Sensor data processing | ~5 ms | Jobs + Burst (IJobParallelFor across 12 streams) |
| Mesh generation (voxel → marching cubes) | ~5 ms | GPU compute shader, dirty-chunk updates only |
| Cutting force calculation | ~1 ms | Burst-compiled Job |
| Robot FK / interpolation | ~1 ms | Burst math on NativeArrays |
| UI dashboard | ~2 ms | UI Toolkit, throttled to 30 Hz updates |
| Rendering (scene + particles + UI) | ~15 ms | URP with SRP Batcher |
| Headroom | ~4 ms | GC, OS, async operations |

### Use URP, not HDRP

**Universal Render Pipeline (URP)** is the right choice. The application is compute-heavy (data processing, mesh generation, signal analysis), and URP's lower rendering overhead preserves GPU budget for compute shaders. It supports VFX Graph, Shader Graph, and all needed visualization features. HDRP's photorealistic rendering is unnecessary overhead for a data-focused digital twin.

### Jobs + Burst for sensor processing and force calculation

Burst compilation delivers **4–15× single-threaded speedup** over standard C# and **17–124× with multi-threaded Jobs** (benchmarked by Unity Technologies on mesh generation workloads). Use `Unity.Mathematics` types (`float3`, `float4x4`, `quaternion`) instead of UnityEngine equivalents—they enable SIMD vectorization under Burst. Keijiro Takahashi's **BurstFFT** demonstrates the pattern for vibration analysis: single-threaded Burst FFT per sensor with parallel processing across sensors via `IJobParallelFor`.

Schedule sensor processing jobs early in the frame and `.Complete()` as late as possible to overlap with rendering. Use `JobHandle.CombineDependencies()` to fan-in multiple independent streams before a fusion step:

```csharp
var imuJob = new IMUProcessJob { data = imuBuffer, output = features }
    .Schedule(12, 1);  // 12 sensors, batch size 1
var forceJob = new ForceCalcJob { voxelDelta = removedVoxels, output = forces }
    .Schedule(imuJob);  // depends on IMU processing
forceJob.Complete();
```

### GPU compute for material removal pipeline

The optimal pipeline keeps data GPU-resident: **tool path → SDF/voxel modification (compute shader) → marching cubes (compute shader) → render (DrawProceduralIndirect)**. No CPU readback is needed for the visual mesh. Use `AsyncGPUReadback` only for scalar values needed on CPU (e.g., removed-voxel count for force calculation, displayed in UI). This avoids the CPU stall caused by `ComputeBuffer.GetData()`.

For the workpiece mesh, use **Mesh API v2** (`Mesh.AllocateWritableMeshData`) when generating on CPU—it provides zero-GC-allocation mesh generation compatible with the Job System. Double-buffer the mesh: render mesh A while generating mesh B, then swap each frame.

### Minimizing garbage collection

Target **zero allocations per frame** in steady state. Use `ConcurrentQueue<T>` (struct-based messages) for thread communication, `NativeArray<T>` with `Allocator.Persistent` for sensor buffers, `StringBuilder` for UI text updates, and Unity's built-in `ObjectPool<T>` (Unity 2021+) for chip fragment GameObjects. Avoid LINQ, string concatenation, and `mesh.vertices` setter (which creates managed arrays) in any per-frame code path. Enable **Incremental GC** to spread collection over frames, preventing spikes.

### VFX Graph for chip particles over legacy Particle System

VFX Graph runs entirely on GPU and can simulate **millions of particles** versus the legacy system's practical limit of thousands. Use mesh rendering mode with pre-modeled curled chip variations. VFX Graph integrates naturally with the material removal pipeline—a GraphicsBuffer can feed spawn parameters (position, velocity, chip size) directly from the cutting zone compute shader. Apply distance-based LOD culling built into VFX Graph for automatic performance scaling.

---

## Conclusion

The architecture centers on **MQTT as the universal data bus** connecting all physical equipment—CNC, robot, and 12 BLE sensors—to a Unity application structured around ScriptableObject event channels and a custom simulation clock. Three decisions are particularly important and non-obvious.

First, **use an nRF52840 gateway for BLE**, not the PC's Bluetooth stack. The Windows BLE limit of ~7 connections and ArduinoBLE's ~40 Hz notification cap are hard constraints that shape the sensor network design. Batch IMU samples or encode as int16 to work within these limits.

Second, **use voxels with GPU marching cubes for material removal**, not mesh CSG. This is the single biggest architectural decision for the cutting simulation—CSG will appear to work initially but will collapse after ~100 boolean operations. The voxel approach scales to millions of operations with zero degradation and naturally supports cutting force estimation via removed-voxel counting.

Third, **build a custom SimulationClock from day one** rather than relying on `Time.timeScale`. The three timing modes (real-time, accelerated, replay) are fundamentally different data-flow patterns, and retrofitting time abstraction later will require rewriting every subsystem. The clock is the spine of the application—all other systems should reference it for time progression.

The combination of Burst-compiled Jobs for CPU computation, GPU compute shaders for voxel operations, and VFX Graph for particle effects distributes the workload across all available hardware. Target 30 FPS with a 33.3 ms frame budget, allocating roughly 5 ms each to sensor processing and mesh generation, 15 ms to rendering, and keeping 4 ms of headroom. Profile continuously with Unity Profiler custom markers on each subsystem to catch regressions early.