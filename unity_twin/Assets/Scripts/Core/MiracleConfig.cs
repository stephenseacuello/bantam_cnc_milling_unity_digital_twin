using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>
    /// ScriptableObject holding MIRACLE system configuration.
    /// Create via Assets > Create > MIRACLE > System Configuration.
    /// Optionally assigned to MiracleBridge; if null, MiracleBridge
    /// uses its own serialized fields as before (no breaking change).
    /// </summary>
    [CreateAssetMenu(fileName = "MiracleConfig", menuName = "MIRACLE/System Configuration")]
    public class MiracleConfig : ScriptableObject
    {
        [Header("ROS Bridge Connection")]
        [Tooltip("IP address of the ROS2 bridge server.")]
        public string rosBridgeIP = "127.0.0.1";

        [Tooltip("TCP port for the Unity-ROS2 bridge.")]
        public int rosBridgePort = 10000;

        [Tooltip("ROS2 Domain ID.")]
        public int rosDomainID = 42;

        [Header("Machine Configuration")]
        [Tooltip("Machine IDs to subscribe to (matches ROS2 topic namespacing).")]
        public string[] machineIds = { "cnc1", "cnc2", "cnc3" };

        [Header("WebSocket / HMI")]
        [Tooltip("WebSocket port for the HMI dashboard bridge.")]
        public int websocketPort = 9090;

        [Header("Timing")]
        [Tooltip("Heartbeat interval in seconds.")]
        public float heartbeatInterval = 0.5f;

        [Tooltip("Keepalive timeout in seconds. Unity marks ROS2 as disconnected after this.")]
        public float keepaliveTimeout = 10f;

        [Tooltip("Reconnection delay in seconds after a connection loss.")]
        public float reconnectDelay = 3f;

        [Header("Performance")]
        [Tooltip("Maximum voxel dirty chunks to process per frame.")]
        public int maxDirtyChunksPerFrame = 8;

        [Tooltip("Maximum triangles per marching cubes chunk.")]
        public int maxTrianglesPerChunk = 32768;

        [Header("Recording")]
        [Tooltip("Default directory for .miracle recording files (relative to StreamingAssets).")]
        public string recordingDirectory = "Recordings";
    }
}
