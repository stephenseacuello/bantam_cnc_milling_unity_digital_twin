# Building a Real-Time Digital Twin of the Bantam Tools CNC in Unity

The Bantam Tools Desktop Explorer CNC can absolutely become a Unity digital twin—the machine runs a **TinyG V9 controller** (not Grbl) that communicates via USB serial at **115,200 baud** with full status query support. For a Unity beginner, the most practical architecture uses an **MQTT middleware with a Python bridge**, avoiding Unity's notoriously tricky serial port handling while maintaining real-time responsiveness.

---

## The Bantam Explorer's communication architecture favors direct serial access

The Bantam Tools Desktop Explorer uses a **Synthetos TinyG V9 motion control board** with an Atmel ATxmega192 processor—this is critical because it's fundamentally different from Grbl-based machines. Bantam runs a forked version of TinyG firmware that supports both text mode and JSON mode communication, making it surprisingly accessible for custom applications despite the proprietary Bantam Tools software.

**Key serial parameters for direct communication:**

| Parameter | Value |
|-----------|-------|
| Baud Rate | 115,200 (configurable up to 230,400) |
| Flow Control | XON/XOFF (default) or RTS/CTS |
| Line Termination | CR, LF, or CR/LF |
| Buffer Size | 254 characters input, 512 output |

The machine responds to **real-time status queries** using the `?` character (no newline required), returning position, velocity, spindle state, and machine status. The response format differs from Grbl—TinyG uses comma-separated fields within pipe-delimited sections:

```
// TinyG status query response format
{"sr":{"posx":10.250,"posy":-5.500,"posz":2.000,"vel":500.0,"stat":5}}
```

Machine state values include: **0**=initializing, **1**=ready, **2**=alarm, **3**=stop, **4**=program end, **5**=running, **6**=hold. You can configure automatic status reports every 250ms during motion using `$sv=2` (verbose mode) and `$si=250` (interval in milliseconds).

**Important caveat:** The Bantam Tools desktop software must NOT be running when you connect directly—the serial port can only have one client. Bantam's software has no public API, so direct serial communication or middleware is your only path to programmatic control.

---

## Unity project setup requires specific packages and configuration

Start with **Unity 2022.3 LTS** or **Unity 6** using the **Universal Render Pipeline (URP)** for cross-platform compatibility—HDRP offers better visuals but demands more powerful hardware and complicates deployment. Create a new 3D URP project from Unity Hub.

**Essential packages and assets to install:**

For MQTT communication (recommended approach):
- **M2MqttUnity** — Free, open-source MQTT client from GitHub ([gpvigano/M2MqttUnity](https://github.com/gpvigano/M2MqttUnity)). Download and copy the M2Mqtt and M2MqttUnity folders into your Assets directory. This handles broker connections, topic subscriptions, and message publishing.

For the 3D model hierarchy and motion:
- **ArticulationBody components** (built into Unity) — Purpose-built for robotic/industrial simulations with prismatic joints for linear axis motion. Far superior to Configurable Joints for CNC applications because they use a reduced-coordinate solver that eliminates constraint drift.

For general industrial simulation patterns (optional but valuable for learning):
- **realvirtual.io Community** — Open-source industrial automation framework providing proven architecture patterns. Available on GitHub at [game4automation/game4automation-Community](https://github.com/game4automation/game4automation-Community).

**Critical Unity configuration for serial communication (if going direct):**
Navigate to Edit → Project Settings → Player → Other Settings and change **Api Compatibility Level** from `.NET Standard 2.0` to `.NET Framework`. This enables `System.IO.Ports` access, though the threading complexity makes MQTT the better choice for beginners.

---

## Your 3D model pipeline starts with existing resources

**GrabCAD has Bantam Tools models available**, including a "Bantam Tools CNC Desktop Milling Machine 4th Axis Housing" and related components. Search GrabCAD's library under the `/library/tag/bantam` and `/library/tag/cnc` tags. You'll need a free account to download. If you can't find the exact Explorer model, Bantam Tools uses Fusion 360 internally—contacting their support for CAD files is worth attempting.

For custom modeling or modifications, the workflow is:

**Fusion 360 → FBX → Unity** using the SimLab FBX Exporter plugin or Autodesk's Unity File Exporter from the App Store. FBX preserves the component hierarchy and parent-child relationships essential for articulated motion. Key export settings: scale factor of **0.01** (Fusion uses mm, Unity uses meters) and enable "Apply Transform" to bake transformations.

**Model hierarchy for proper axis motion:**
```
CNC_Machine (root GameObject)
├── Base_Frame (static)
├── X_Axis_Assembly
│   └── X_Carriage (moves along X)
│       └── Y_Axis_Assembly
│           └── Y_Bed (moves along Y)
│               └── Z_Axis_Assembly
│                   └── Z_Spindle_Mount (moves along Z)
│                       └── Spindle_Motor
│                           └── Tool_Holder
└── Enclosure (static)
```

This nesting ensures that when the X-axis moves, all attached Y and Z components move with it automatically—no additional scripting required for dependent motion.

**Setting up ArticulationBody prismatic joints:**

```csharp
// Attach to each axis GameObject (X, Y, Z carriages)
ArticulationBody xAxis = GetComponent<ArticulationBody>();
xAxis.jointType = ArticulationJointType.PrismaticJoint;

// Configure drive for position control
var drive = xAxis.xDrive;
drive.lowerLimit = 0f;           // Minimum travel (mm)
drive.upperLimit = 200f;         // Maximum travel (Explorer X-axis)
drive.stiffness = 100000f;       // High stiffness = rigid motion
drive.damping = 10000f;          // Prevents oscillation
drive.forceLimit = float.MaxValue;
xAxis.xDrive = drive;
```

To move an axis to a target position:
```csharp
var drive = articulationBody.xDrive;
drive.target = targetPositionInMM;
articulationBody.xDrive = drive;
```

---

## The recommended communication architecture uses MQTT middleware

Direct serial communication from Unity is technically possible but fraught with threading issues, port cleanup problems, and cross-platform inconsistencies. The **MQTT broker pattern** with a Python bridge provides cleaner separation of concerns and is significantly easier to debug.

**Architecture overview:**
```
┌──────────────────────────┐
│   Unity (M2MqttUnity)    │  Subscribes: cnc/position, cnc/status
│   - 3D visualization     │  Publishes: cnc/command
│   - UI controls          │
└───────────┬──────────────┘
            │ MQTT (localhost:1883)
            ▼
┌──────────────────────────┐
│   Mosquitto MQTT Broker  │  Lightweight, runs locally
└───────────┬──────────────┘
            │ MQTT
            ▼
┌──────────────────────────┐
│   Python Bridge Script   │  Handles TinyG protocol
│   - pyserial (serial)    │  Polls status, forwards commands
│   - paho-mqtt (broker)   │
└───────────┬──────────────┘
            │ USB Serial (115200 baud)
            ▼
┌──────────────────────────┐
│   Bantam CNC Machine     │
│   TinyG V9 Controller    │
└──────────────────────────┘
```

**MQTT topic structure for the digital twin:**
```
cnc/position/x          # Float: current X position in mm
cnc/position/y          # Float: current Y position in mm  
cnc/position/z          # Float: current Z position in mm
cnc/status/state        # String: Idle, Run, Hold, Alarm
cnc/status/spindle      # Float: spindle RPM (0 if off)
cnc/status/feedrate     # Float: current feed rate
cnc/command             # String: G-code commands from Unity
```

**Python bridge script (core structure):**

```python
import serial
import json
import paho.mqtt.client as mqtt
import time
import threading

class TinyGBridge:
    def __init__(self, port='COM3', broker='localhost'):
        self.serial = serial.Serial(port, 115200, timeout=0.1)
        self.mqtt = mqtt.Client()
        self.mqtt.connect(broker, 1883)
        self.mqtt.subscribe("cnc/command")
        self.mqtt.on_message = self.on_command
        self.running = True
        
    def on_command(self, client, userdata, msg):
        # Forward G-code to TinyG
        gcode = msg.payload.decode()
        self.serial.write(f'{gcode}\n'.encode())
        
    def poll_status(self):
        while self.running:
            self.serial.write(b'{"sr":null}\n')  # JSON status query
            time.sleep(0.1)  # 10 Hz polling
            
            while self.serial.in_waiting:
                line = self.serial.readline().decode().strip()
                if line.startswith('{"sr"'):
                    status = json.loads(line)['sr']
                    self.mqtt.publish('cnc/position/x', status.get('posx', 0))
                    self.mqtt.publish('cnc/position/y', status.get('posy', 0))
                    self.mqtt.publish('cnc/position/z', status.get('posz', 0))
                    self.mqtt.publish('cnc/status/state', status.get('stat', 0))
                    
    def run(self):
        threading.Thread(target=self.poll_status).start()
        self.mqtt.loop_forever()
```

Install dependencies with: `pip install pyserial paho-mqtt`

**Unity-side MQTT subscription (C#):**

```csharp
using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Text;

public class CNCDigitalTwin : M2MqttUnityClient
{
    public ArticulationBody xAxis;
    public ArticulationBody yAxis;
    public ArticulationBody zAxis;
    
    private float targetX, targetY, targetZ;
    private Queue<System.Action> mainThreadActions = new Queue<System.Action>();

    protected override void Start()
    {
        brokerAddress = "localhost";
        brokerPort = 1883;
        base.Start();
    }

    protected override void SubscribeTopics()
    {
        client.Subscribe(new string[] { 
            "cnc/position/x", 
            "cnc/position/y", 
            "cnc/position/z" 
        }, new byte[] { 
            MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE,
            MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE,
            MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE
        });
    }

    protected override void DecodeMessage(string topic, byte[] message)
    {
        float value = float.Parse(Encoding.UTF8.GetString(message), 
            System.Globalization.CultureInfo.InvariantCulture);
        
        // Queue for main thread execution (Unity is not thread-safe)
        mainThreadActions.Enqueue(() => {
            switch (topic)
            {
                case "cnc/position/x": UpdateAxis(xAxis, value); break;
                case "cnc/position/y": UpdateAxis(yAxis, value); break;
                case "cnc/position/z": UpdateAxis(zAxis, value); break;
            }
        });
    }

    void UpdateAxis(ArticulationBody axis, float position)
    {
        var drive = axis.xDrive;
        drive.target = position / 1000f;  // Convert mm to Unity meters
        axis.xDrive = drive;
    }

    void Update()
    {
        while (mainThreadActions.Count > 0)
            mainThreadActions.Dequeue().Invoke();
    }

    public void SendGCode(string command)
    {
        client.Publish("cnc/command", Encoding.UTF8.GetBytes(command));
    }
}
```

---

## Beginner learning roadmap with specific resources

**Phase 1: Unity fundamentals (1-2 weeks)**
- Complete Unity's official "Create with Code" course on Unity Learn—free and comprehensive
- Focus specifically on: GameObjects, Transform component (position/rotation/scale), parent-child hierarchies, C# scripting basics, and the Inspector window
- Key concept to internalize: Unity's coordinate system is **Y-up** and uses **meters**, not millimeters

**Phase 2: Basic 3D model and hierarchy setup (3-5 days)**
- Download a CNC model from GrabCAD or create a simplified box-geometry placeholder
- Import as FBX, verify scale (multiply by 0.001 if imported in mm)
- Create the nested hierarchy manually if needed: right-click in Hierarchy → Create Empty, then drag components to establish parent-child relationships
- Add ArticulationBody to each moving axis, experiment with prismatic joint settings

**Phase 3: Communication layer (1 week)**
- Install Mosquitto MQTT broker (download from mosquitto.org, runs as a Windows service)
- Test broker with MQTT Explorer desktop client (free, visual debugging tool)
- Create Python bridge script, test independently before Unity integration
- Import M2MqttUnity into Unity, start with included example scene

**Phase 4: Integration and testing (1 week)**
- Connect Unity to MQTT broker, verify subscription messages arrive
- Map received position data to ArticulationBody targets
- Create simple UI (Unity's UI Toolkit or legacy UI) for G-code input field and send button
- Test with simulated data before connecting real machine

**Phase 5: Real machine integration (ongoing)**
- **Always test G-code in Bantam's official software first**
- Start with status monitoring only—no motion commands
- Add emergency stop UI element that publishes feedhold command (`!`)
- Gradually add jog controls and G-code streaming

**Essential Unity concepts to master:**
1. **Transform.position vs ArticulationBody.xDrive.target** — Use ArticulationBody for physics-driven motion, direct Transform manipulation for instant teleportation
2. **Update() vs FixedUpdate()** — Process MQTT messages in Update(), physics in FixedUpdate()
3. **Thread safety** — Never call Unity API from background threads; use a queue pattern
4. **SerializeField attribute** — Expose private variables in Inspector without making them public

---

## Alternative approaches and existing projects worth examining

**No turnkey open-source CNC digital twin exists for Unity**, but several projects provide valuable architecture patterns:

**realvirtual.io** (formerly Game4Automation) offers a complete industrial automation framework with OPC UA, MQTT, and Siemens PLC interfaces. While focused on conveyors and factory automation, the component architecture directly applies to CNC machines. Study their GitHub repository for patterns around signal handling and motion control.

**Unity-Technologies/Unity-Robotics-Hub** demonstrates ROS integration with Unity, including articulation body setup for robotic arms. The pick-and-place tutorial shows industrial-grade motion control that transfers well to CNC applications.

**rparak's Unity3D_Robotics_Overview** on GitHub contains working digital twins for UR3 robots and a sorting machine using OPC UA—the sorting machine project is closest to CNC architecture and demonstrates real-time TCP/IP communication patterns.

**For direct serial (if you skip MQTT):**
The **Ardity** plugin (free, open-source at ardity.dwilches.com) provides a thread-safe queue implementation for Unity serial communication. If you choose this path, you'll need to adapt the TinyG JSON parsing for Unity's main thread execution model.

**Why MQTT beats ROS for this project:**
ROS (Robot Operating System) is powerful but requires Linux (or WSL/Docker), has a steep learning curve, and is overkill for a single-machine digital twin. MQTT runs natively on Windows, has excellent Python and C# libraries, and the pub/sub model naturally fits the monitoring use case. Save ROS for when you're integrating multiple machines or need advanced robotics features.

---

## Conclusion

Building a Bantam Explorer digital twin is achievable for a Unity beginner within **4-6 weeks** of focused learning. The TinyG controller's JSON mode and status reporting make it surprisingly accessible despite Bantam's proprietary software—you're essentially building a custom G-code sender with 3D visualization.

Start with the MQTT architecture to isolate serial communication complexity in Python, where debugging is straightforward. Get position data flowing to a static cube in Unity before investing in a detailed 3D model. The ArticulationBody system with prismatic joints handles the physics correctly without manual interpolation code.

The critical safety consideration: always implement an emergency stop that sends the feedhold character (`!`) to the machine. Never trust software alone for CNC safety—the physical machine's door interlock and e-stop remain your primary protection.