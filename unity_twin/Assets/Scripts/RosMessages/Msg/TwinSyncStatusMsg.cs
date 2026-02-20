using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class TwinSyncStatusMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/TwinSyncStatus";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public double sync_quality;
        public double drift_magnitude;
        public double[] axis_drift;
        public double sync_latency_ms;
        public bool correction_active;
        public uint corrections_applied;

        public TwinSyncStatusMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.sync_quality = 0.0;
            this.drift_magnitude = 0.0;
            this.axis_drift = new double[0];
            this.sync_latency_ms = 0.0;
            this.correction_active = false;
            this.corrections_applied = 0;
        }

        public TwinSyncStatusMsg(
            TimeMsg timestamp,
            string machine_id,
            double sync_quality,
            double drift_magnitude,
            double[] axis_drift,
            double sync_latency_ms,
            bool correction_active,
            uint corrections_applied)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.sync_quality = sync_quality;
            this.drift_magnitude = drift_magnitude;
            this.axis_drift = axis_drift;
            this.sync_latency_ms = sync_latency_ms;
            this.correction_active = correction_active;
            this.corrections_applied = corrections_applied;
        }

        public static TwinSyncStatusMsg Deserialize(MessageDeserializer deserializer) => new TwinSyncStatusMsg(deserializer);

        private TwinSyncStatusMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.sync_quality);
            deserializer.Read(out this.drift_magnitude);
            deserializer.Read(out this.axis_drift, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.sync_latency_ms);
            deserializer.Read(out this.correction_active);
            deserializer.Read(out this.corrections_applied);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.sync_quality);
            serializer.Write(this.drift_magnitude);
            serializer.WriteLength(this.axis_drift);
            serializer.Write(this.axis_drift);
            serializer.Write(this.sync_latency_ms);
            serializer.Write(this.correction_active);
            serializer.Write(this.corrections_applied);
        }

        public override string ToString()
        {
            return "TwinSyncStatusMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nsync_quality: " + sync_quality.ToString() +
            "\ndrift_magnitude: " + drift_magnitude.ToString() +
            "\naxis_drift: " + System.String.Join(", ", axis_drift.ToList()) +
            "\nsync_latency_ms: " + sync_latency_ms.ToString() +
            "\ncorrection_active: " + correction_active.ToString() +
            "\ncorrections_applied: " + corrections_applied.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
