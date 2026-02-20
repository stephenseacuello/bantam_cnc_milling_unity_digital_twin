using System.Collections;
using UnityEngine;
using MiracleTwin.Core;

namespace MiracleTwin.Robots
{
    /// <summary>
    /// Generic 6-DOF robot controller driven by URDF ArticulationBody joints.
    /// Receives joint state from ROS2 or local IK and smoothly interpolates.
    /// Works for both Niryo Ned2 and xArm 6 Lite.
    ///
    /// Joint discovery runs in Awake, with a retry in Start and a 1-frame
    /// coroutine fallback for runtime-added URDF components.
    /// </summary>
    public class RobotController : MonoBehaviour
    {
        [Header("Identity")]
        [SerializeField] private string robotId = "ned2";
        [SerializeField] private RobotJointStateEventSO jointStateEvent;

        [Header("Joint Configuration")]
        [SerializeField] private ArticulationBody[] joints = new ArticulationBody[6];
        [SerializeField] private float[] jointLimitsLower = new float[6];
        [SerializeField] private float[] jointLimitsUpper = new float[6];

        [Header("Motion")]
        [SerializeField, Range(0.05f, 0.5f)] private float interpolationSpeed = 0.3f;
        [SerializeField] private bool useGravity = false;

        public string RobotId => robotId;
        public float[] CurrentJoints { get; private set; } = new float[6];
        public float[] TargetJoints { get; private set; } = new float[6];
        public bool IsMoving { get; private set; }

        /// <summary>True when this robot has received at least one ROS joint command.</summary>
        public bool IsRosControlled { get; private set; }

        private float[] previousJoints = new float[6];
        private bool jointsDiscovered;

        void Awake()
        {
            TryDiscoverJoints("Awake");
        }

        void Start()
        {
            // Retry in Start — runtime-added components may not be ready in Awake
            if (!jointsDiscovered)
            {
                TryDiscoverJoints("Start");
            }

            // Last resort: wait one frame for URDF importer to finish
            if (!jointsDiscovered)
            {
                StartCoroutine(RetryDiscoveryNextFrame());
            }
        }

        private IEnumerator RetryDiscoveryNextFrame()
        {
            yield return null; // wait one frame
            if (!jointsDiscovered)
            {
                TryDiscoverJoints("Coroutine(+1frame)");
                if (!jointsDiscovered)
                    Debug.LogWarning($"[RobotController] {robotId}: No joints found after all retries. " +
                        "Robot will not move. Check URDF import or assign joints manually.");
            }
        }

        /// <summary>
        /// Auto-discover ArticulationBody joints from child hierarchy.
        /// Collects up to 6 revolute joints in hierarchy order (depth-first).
        /// Also configures xDrive stiffness/damping and disables gravity.
        /// </summary>
        private void TryDiscoverJoints(string caller)
        {
            // Respect manual assignment — skip if any joint is already set
            for (int i = 0; i < joints.Length; i++)
            {
                if (joints[i] != null)
                {
                    jointsDiscovered = true;
                    // Auto-populate limits from existing joints if still at defaults
                    PopulateJointLimitsFromDrives();
                    return;
                }
            }

            var allBodies = GetComponentsInChildren<ArticulationBody>();
            int jointIndex = 0;

            // Log all ArticulationBodies for debugging
            Debug.Log($"[RobotController] {robotId} ({caller}): Found {allBodies.Length} ArticulationBodies:");
            foreach (var ab in allBodies)
            {
                Debug.Log($"  → {ab.name}: type={ab.jointType}, isRoot={ab.isRoot}");
            }

            // Ensure root ArticulationBody is immovable — required for joint chain to articulate
            foreach (var ab in allBodies)
            {
                if (ab.isRoot)
                {
                    ab.immovable = true;
                    if (!useGravity) ab.useGravity = false;
                    break;
                }
            }

            // Pass 1: RevoluteJoint (most common for 6-DOF arms)
            foreach (var ab in allBodies)
            {
                if (jointIndex >= 6) break;
                if (ab.isRoot) continue;
                if (ab.jointType != ArticulationJointType.RevoluteJoint) continue;

                joints[jointIndex] = ab;
                ConfigureDrive(ab);
                jointIndex++;
            }

            // Pass 2: if no revolute joints, try any non-Fixed joint
            if (jointIndex == 0)
            {
                foreach (var ab in allBodies)
                {
                    if (jointIndex >= 6) break;
                    if (ab.isRoot) continue;
                    if (ab.jointType == ArticulationJointType.FixedJoint) continue;

                    joints[jointIndex] = ab;
                    ConfigureDrive(ab);
                    jointIndex++;
                }
            }

            if (jointIndex > 0)
            {
                jointsDiscovered = true;
                PopulateJointLimitsFromDrives();
                Debug.Log($"[RobotController] {robotId} ({caller}): Auto-discovered {jointIndex} joints");
            }
        }

        /// <summary>
        /// Auto-populate joint limits from ArticulationBody drive limits.
        /// Falls back to generous defaults (±2π) if drive limits are zero.
        /// </summary>
        private void PopulateJointLimitsFromDrives()
        {
            for (int i = 0; i < 6; i++)
            {
                // Skip if already manually configured (non-zero range)
                if (jointLimitsLower[i] != 0f || jointLimitsUpper[i] != 0f) continue;
                if (joints[i] == null) continue;

                var drive = joints[i].xDrive;
                if (drive.lowerLimit < drive.upperLimit)
                {
                    // Use ArticulationBody limits (in degrees), convert to radians
                    jointLimitsLower[i] = drive.lowerLimit * Mathf.Deg2Rad;
                    jointLimitsUpper[i] = drive.upperLimit * Mathf.Deg2Rad;
                }
                else
                {
                    // No meaningful limits set — use generous defaults for 6-DOF arms
                    jointLimitsLower[i] = -2f * Mathf.PI;
                    jointLimitsUpper[i] = 2f * Mathf.PI;
                }
            }
        }

        private void ConfigureDrive(ArticulationBody ab)
        {
            var drive = ab.xDrive;
            // High stiffness for responsive position control,
            // high damping to prevent overshoot/bouncing (near critically damped).
            drive.stiffness = 50000f;
            drive.damping = 5000f;
            drive.forceLimit = float.MaxValue;
            ab.xDrive = drive;

            // Disable gravity for visualization (prevents sagging)
            if (!useGravity)
                ab.useGravity = false;
        }

        void OnEnable()
        {
            if (jointStateEvent != null)
                jointStateEvent.Register(OnJointState);

            // Disable gravity on all links
            if (!useGravity)
            {
                foreach (var joint in joints)
                {
                    if (joint != null)
                        joint.useGravity = false;
                }
            }
        }

        void OnDisable()
        {
            if (jointStateEvent != null)
                jointStateEvent.Unregister(OnJointState);
        }

        private void OnJointState(RobotJointStateData data)
        {
            if (data.robotId != robotId) return;

            if (!IsRosControlled)
            {
                IsRosControlled = true;
                Debug.Log($"[RobotController] {robotId}: Now under ROS control — demo motion disabled.");
            }

            for (int i = 0; i < 6 && i < data.positions.Length; i++)
            {
                TargetJoints[i] = Mathf.Clamp(
                    (float)data.positions[i],
                    jointLimitsLower[i],
                    jointLimitsUpper[i]
                );
            }
        }

        private float publishTimer;
        private const float JOINT_PUBLISH_INTERVAL = 0.1f; // 10 Hz

        void FixedUpdate()
        {
            IsMoving = false;
            for (int i = 0; i < 6; i++)
            {
                previousJoints[i] = CurrentJoints[i];
                CurrentJoints[i] = Mathf.Lerp(CurrentJoints[i], TargetJoints[i], interpolationSpeed);

                if (Mathf.Abs(CurrentJoints[i] - TargetJoints[i]) > 0.001f)
                    IsMoving = true;

                if (joints[i] != null)
                {
                    var drive = joints[i].xDrive;
                    drive.target = CurrentJoints[i] * Mathf.Rad2Deg;
                    joints[i].xDrive = drive;
                }
            }

            // Publish joint state to ROS2 at lower rate
            publishTimer += Time.fixedDeltaTime;
            if (publishTimer >= JOINT_PUBLISH_INTERVAL)
            {
                publishTimer = 0;
                if (MiracleBridge.Instance != null && MiracleBridge.Instance.IsConnected)
                    MiracleBridge.Instance.PublishRobotJointState(GetJointStateData());
            }
        }

        /// <summary>Set joint targets directly (radians).</summary>
        public void SetJointTargets(float[] targets)
        {
            for (int i = 0; i < 6 && i < targets.Length; i++)
                TargetJoints[i] = targets[i];
        }

        /// <summary>Immediately snap to target joints without interpolation.</summary>
        public void SnapToTargets()
        {
            for (int i = 0; i < 6; i++)
            {
                CurrentJoints[i] = TargetJoints[i];
                if (joints[i] != null)
                {
                    var drive = joints[i].xDrive;
                    drive.target = CurrentJoints[i] * Mathf.Rad2Deg;
                    joints[i].xDrive = drive;
                }
            }
        }

        /// <summary>Move all joints to zero (home position).</summary>
        public void GoHome()
        {
            for (int i = 0; i < 6; i++)
                TargetJoints[i] = 0f;
        }

        public Transform GetEndEffector()
        {
            if (joints[5] != null)
                return joints[5].transform;
            return transform;
        }

        /// <summary>
        /// Get current joint state as a RobotJointStateData struct for publishing.
        /// Computes velocities from difference between current and previous joint positions.
        /// </summary>
        public RobotJointStateData GetJointStateData()
        {
            double[] positions = new double[6];
            double[] velocities = new double[6];
            float dt = Time.fixedDeltaTime;

            for (int i = 0; i < 6; i++)
            {
                positions[i] = CurrentJoints[i];
                velocities[i] = dt > 0 ? (CurrentJoints[i] - previousJoints[i]) / dt : 0;
            }

            return new RobotJointStateData
            {
                robotId = robotId,
                positions = positions,
                velocities = velocities,
                gripperState = "OPEN"
            };
        }
    }
}
