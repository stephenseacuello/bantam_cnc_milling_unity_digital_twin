using UnityEngine;
using RosMessageTypes.Miracle;
using MiracleTwin.Core;
using MiracleTwin.CNC;
using MiracleTwin.Robots;
using MiracleTwin.Cutting;

namespace MiracleTwin.Testing
{
    /// <summary>
    /// Local test driver that simulates the full MIRACLE cell WITHOUT requiring ROS2.
    /// Raises event SOs with synthetic data, animates CNC root with vibration,
    /// and drives robot arm joints through demo motions.
    /// Enable to demo the pipeline; disable when using real ROS2 connection.
    /// </summary>
    public class LocalCNCTestDriver : MonoBehaviour
    {
        [Header("Event Channels")]
        [SerializeField] private MachineStateEventSO machineStateEvent;
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private SystemKPIsEventSO systemKPIsEvent;

        [Header("Simulation")]
        [SerializeField] private float publishRate = 20f;

        [Header("CNC Animation")]
        [Tooltip("Machining vibration amplitude in meters")]
        [SerializeField] private float vibrationAmplitude = 0.0005f;

        [Header("Robot Animation")]
        [Tooltip("Enable demo motion on robot arms")]
        [SerializeField] private bool animateRobots = true;
        [Tooltip("Robot joint sweep amplitude in radians (1.0 = ~57 degrees)")]
        [SerializeField] private float robotJointAmplitude = 1.2f;

        private float timer;
        private float elapsed;
        private bool startupLogged;

        // Auto-discovered CNC machines (for root vibration only — axis motion via controllers)
        private Transform bantamRoot;
        private Transform crRoot;
        private Vector3 bantamBasePos;
        private Vector3 crBasePos;

        // Auto-discovered robots
        private RobotController[] robots;

        // When CuttingSimulationManager is active, skip fake cutting data
        private bool physicsSimActive;

        void Start()
        {
            DisableURDFImporterComponents();
            FindCNCMachines();
            EnsureRobotControllers();
            FindRobots();

            // Detect if real cutting physics pipeline is wired
            var simManager = Object.FindFirstObjectByType<CuttingSimulationManager>();
            if (simManager != null)
            {
                physicsSimActive = true;
                Debug.Log("[LocalCNCTestDriver] CuttingSimulationManager detected — real physics pipeline active. " +
                    "Skipping fake cutting data; MachineState + SystemKPIs still publishing.");
            }

            Debug.Log($"[LocalCNCTestDriver] Active. " +
                $"Bantam: {(bantamRoot != null ? "found" : "missing")}, " +
                $"CR-1: {(crRoot != null ? "found" : "missing")}, " +
                $"Robots: {(robots != null ? robots.Length : 0)}, " +
                $"EventSO: {(machineStateEvent != null ? "wired" : "NULL — run MIRACLE > Wire Dashboard!")}");
        }

        private void DisableURDFImporterComponents()
        {
            int disabled = 0;
            foreach (var mb in Object.FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                string typeName = mb.GetType().Name;
                // Disable URDF importer scripts that interfere with our RobotController
                if (typeName == "FKRobot" || typeName == "Controller")
                {
                    mb.enabled = false;
                    disabled++;
                }
            }
            if (disabled > 0)
                Debug.Log($"[LocalCNCTestDriver] Disabled {disabled} URDF importer component(s) (FKRobot + Controller).");
        }

        private void EnsureRobotControllers()
        {
            string[] robotNames = { "lite6", "niryo_ned2", "xarm6", "xArm6" };
            string[] robotIds = { "lite6", "ned2", "xarm6", "xarm6" };

            for (int i = 0; i < robotNames.Length; i++)
            {
                var go = GameObject.Find(robotNames[i]);
                if (go == null) continue;

                var ctrl = go.GetComponent<RobotController>();
                if (ctrl == null)
                {
                    ctrl = go.AddComponent<RobotController>();
                    var field = typeof(RobotController).GetField("robotId",
                        System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                    if (field != null)
                        field.SetValue(ctrl, robotIds[i]);
                    Debug.Log($"[LocalCNCTestDriver] Added RobotController to '{robotNames[i]}' (robotId: {robotIds[i]})");
                }
            }
        }

        private void FindCNCMachines()
        {
            string[] bantamNames = { "BantamExplorer_CNC", "BantamExplorer", "Bantam_CNC", "BantamExplorerCNC" };
            foreach (var name in bantamNames)
            {
                var go = GameObject.Find(name);
                if (go != null) { bantamRoot = go.transform; break; }
            }
            if (bantamRoot != null)
                bantamBasePos = bantamRoot.localPosition;

            string[] crNames = { "CoastRunnerCR1_CNC", "CoastRunnerCR1", "CoastRunner_CNC" };
            foreach (var name in crNames)
            {
                var go = GameObject.Find(name);
                if (go != null) { crRoot = go.transform; break; }
            }
            if (crRoot != null)
                crBasePos = crRoot.localPosition;
        }

        private void FindRobots()
        {
            robots = Object.FindObjectsByType<RobotController>(FindObjectsSortMode.None);
        }

        void Update()
        {
            timer += Time.deltaTime;
            float interval = 1f / publishRate;
            if (timer < interval) return;
            timer -= interval;
            elapsed += interval;

            if (!MiracleBridge.RosMachineStateActive)
            {
                PublishMachineState();
                AnimateBantam();
                AnimateCoastRunner();
            }
            if (!physicsSimActive)
                PublishCuttingState();
            PublishSystemKPIs();

            if (animateRobots)
                AnimateRobots();

            // One-time confirmation log after first tick
            if (!startupLogged && elapsed > 0.1f)
            {
                startupLogged = true;
                int robotsMoving = 0;
                if (robots != null)
                    foreach (var r in robots)
                        if (r != null && r.IsMoving) robotsMoving++;
                Debug.Log($"[LocalCNCTestDriver] First tick confirmed. Robots moving: {robotsMoving}/{(robots != null ? robots.Length : 0)}");
            }
        }

        private void PublishMachineState()
        {
            if (machineStateEvent == null) return;

            // Bantam Explorer (cnc1) — spiral toolpath within 152x102mm work envelope
            PublishMachineStateFor("cnc1", "demo_spiral.nc", 12000.0, 800.0,
                5f, 147f, 5f, 96f, 2f, 65f, 76.2f, 50.8f, 20f, 0f);

            // CoastRunner CR-1 (cnc2) — rectangular pocket within 89x242mm work envelope
            PublishMachineStateFor("cnc2", "pocket_rect.nc", 10000.0, 600.0,
                2f, 85f, 5f, 237f, 2f, 75f, 44.5f, 121f, 15f, Mathf.PI * 0.5f);
        }

        private void PublishMachineStateFor(string machineId, string program,
            double rpm, double feed,
            float xMin, float xMax, float yMin, float yMax, float zMin, float zMax,
            float xCenter, float yCenter, float period, float phaseOffset)
        {
            var msg = new MachineStateMsg();
            msg.machine_id = machineId;
            msg.status = "RUNNING";
            msg.current_program = program;
            msg.spindle_speed = rpm;
            msg.feed_rate = feed;

            float angle = (elapsed / period) * 2f * Mathf.PI + phaseOffset;
            float radius = (xMax - xMin) * 0.15f *
                (0.3f + 0.7f * Mathf.Abs(Mathf.Sin(elapsed / 40f * Mathf.PI)));

            float x = Mathf.Clamp(xCenter + radius * Mathf.Cos(angle), xMin, xMax);
            float y = Mathf.Clamp(yCenter + radius * Mathf.Sin(angle), yMin, yMax);
            float z = Mathf.Clamp((zMin + zMax) * 0.3f + (zMax - zMin) * 0.2f *
                Mathf.Sin(elapsed / 4f * Mathf.PI), zMin, zMax);

            msg.axis_positions = new double[] { x, y, z };
            msg.axis_velocities = new double[] {
                -radius * Mathf.Sin(angle) * (2f * Mathf.PI / Mathf.Max(period, 1f)),
                radius * Mathf.Cos(angle) * (2f * Mathf.PI / Mathf.Max(period, 1f)),
                (zMax - zMin) * 0.2f * Mathf.Cos(elapsed / 4f * Mathf.PI) * (Mathf.PI / 4f)
            };

            msg.spindle_load = 35.0 + 15.0 * Mathf.Sin(elapsed / 3f + phaseOffset);
            msg.coolant_level = 85.0;
            msg.current_line = (uint)(elapsed * 10f) % 500;
            msg.cycle_time_elapsed = elapsed;
            msg.cycle_time_remaining = Mathf.Max(0f, 300f - elapsed);

            machineStateEvent.Raise(msg);
        }

        private void PublishCuttingState()
        {
            if (cuttingStateEvent == null) return;

            float cutPhase = elapsed * 0.5f;
            var state = new CuttingStateData
            {
                spindleRPM = 12000f,
                feedRate = 800f,
                toolPosition = new Vector3(76.2f, 50.8f, 15f),
                toolDirection = Vector3.down,
                axialDepth = 1.5f,
                radialDepth = 3.0f,
                toolDiameter = 6.35f,
                isCutting = true,
                currentGCodeLine = (int)((elapsed * 10f) % 500),

                forceFx = 45f + 20f * Mathf.Sin(cutPhase * 2.1f),
                forceFy = 30f + 15f * Mathf.Sin(cutPhase * 1.7f + 0.5f),
                forceFz = 60f + 25f * Mathf.Sin(cutPhase * 2.5f + 1.0f),
                powerWatts = 180f + 60f * Mathf.Sin(cutPhase),
                torqueNm = 0.8f + 0.3f * Mathf.Sin(cutPhase * 1.3f),
                mrr = 12.5f + 4f * Mathf.Sin(cutPhase * 0.9f),

                toolTemperature = 35f + 15f * (1f - Mathf.Exp(-elapsed / 120f)),
                interfaceTemperature = 120f + 40f * (1f - Mathf.Exp(-elapsed / 90f)),

                flankWearVB = 0.02f + 0.08f * (elapsed / 600f),
                wearPercentage = Mathf.Clamp01(elapsed / 600f) * 100f,

                chipThicknessRatio = 2.1f,
                shearAngleDeg = 32f,
                chipCurlRadius = 0.8f,
                chipVelocity = 3.2f
            };

            cuttingStateEvent.Raise(state);
        }

        private void PublishSystemKPIs()
        {
            if (systemKPIsEvent == null) return;

            if (elapsed % 2f > 1f / publishRate) return;

            var msg = new SystemKPIsMsg();
            msg.oee = 78.5 + 5.0 * Mathf.Sin(elapsed / 60f);
            msg.availability = 92.0 + 3.0 * Mathf.Sin(elapsed / 45f);
            msg.performance = 85.0 + 4.0 * Mathf.Sin(elapsed / 50f + 1f);
            msg.quality = 98.5 + 1.0 * Mathf.Sin(elapsed / 80f + 2f);
            msg.cpk = 1.45 + 0.15 * Mathf.Sin(elapsed / 100f);
            msg.mtbf = 240.0;
            msg.mttr = 15.0;
            msg.energy_efficiency = 0.82 + 0.05 * Mathf.Sin(elapsed / 70f);
            msg.schedule_adherence = 95.0;
            msg.tool_life_utilization = 65.0 + elapsed / 600f * 20f;
            msg.jobs_completed_today = (uint)(elapsed / 60f);
            msg.jobs_in_progress = 1;
            msg.jobs_queued = 3;

            systemKPIsEvent.Raise(msg);
        }

        private void AnimateBantam()
        {
            if (bantamRoot == null) return;

            // Machining vibration on the root body (subtle)
            float vibFreq = 120f;
            float vib = vibrationAmplitude * Mathf.Sin(elapsed * vibFreq * Mathf.PI);
            bantamRoot.localPosition = bantamBasePos + new Vector3(vib, vib * 0.5f, vib);
        }

        private void AnimateCoastRunner()
        {
            if (crRoot == null) return;

            float vib = vibrationAmplitude * 0.4f * Mathf.Sin(elapsed * 60f * Mathf.PI);
            crRoot.localPosition = crBasePos + new Vector3(vib, 0f, vib);
        }

        private void AnimateRobots()
        {
            if (robots == null) return;

            for (int r = 0; r < robots.Length; r++)
            {
                if (robots[r] == null || !robots[r].enabled) continue;
                if (robots[r].IsRosControlled) continue; // ROS has taken over — don't overwrite

                float phaseOffset = r * Mathf.PI * 0.5f;
                float[] targets = new float[6];

                targets[0] = robotJointAmplitude * 0.8f * Mathf.Sin(elapsed * 0.5f + phaseOffset);
                targets[1] = robotJointAmplitude * 0.6f * Mathf.Sin(elapsed * 0.7f + phaseOffset + 0.5f);
                targets[2] = robotJointAmplitude * 0.7f * Mathf.Sin(elapsed * 0.9f + phaseOffset + 1.0f);
                targets[3] = robotJointAmplitude * 0.5f * Mathf.Sin(elapsed * 1.1f + phaseOffset + 1.5f);
                targets[4] = robotJointAmplitude * 0.4f * Mathf.Sin(elapsed * 1.4f + phaseOffset + 2.0f);
                targets[5] = robotJointAmplitude * 0.3f * Mathf.Sin(elapsed * 1.8f + phaseOffset + 2.5f);

                robots[r].SetJointTargets(targets);
            }
        }
    }
}
