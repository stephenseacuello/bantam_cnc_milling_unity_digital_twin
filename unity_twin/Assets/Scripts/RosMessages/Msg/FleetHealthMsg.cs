using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class FleetHealthMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/FleetHealth";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public uint total_nodes;
        public uint healthy_nodes;
        public uint degraded_nodes;
        public uint failed_nodes;
        public uint critical_healthy;
        public uint critical_total;
        public double health_score;
        public string[] failed_node_names;
        public string[] degraded_node_names;

        public FleetHealthMsg()
        {
            this.timestamp = new TimeMsg();
            this.total_nodes = 0;
            this.healthy_nodes = 0;
            this.degraded_nodes = 0;
            this.failed_nodes = 0;
            this.critical_healthy = 0;
            this.critical_total = 0;
            this.health_score = 0.0;
            this.failed_node_names = new string[0];
            this.degraded_node_names = new string[0];
        }

        public FleetHealthMsg(
            TimeMsg timestamp,
            uint total_nodes,
            uint healthy_nodes,
            uint degraded_nodes,
            uint failed_nodes,
            uint critical_healthy,
            uint critical_total,
            double health_score,
            string[] failed_node_names,
            string[] degraded_node_names)
        {
            this.timestamp = timestamp;
            this.total_nodes = total_nodes;
            this.healthy_nodes = healthy_nodes;
            this.degraded_nodes = degraded_nodes;
            this.failed_nodes = failed_nodes;
            this.critical_healthy = critical_healthy;
            this.critical_total = critical_total;
            this.health_score = health_score;
            this.failed_node_names = failed_node_names;
            this.degraded_node_names = degraded_node_names;
        }

        public static FleetHealthMsg Deserialize(MessageDeserializer deserializer) => new FleetHealthMsg(deserializer);

        private FleetHealthMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.total_nodes);
            deserializer.Read(out this.healthy_nodes);
            deserializer.Read(out this.degraded_nodes);
            deserializer.Read(out this.failed_nodes);
            deserializer.Read(out this.critical_healthy);
            deserializer.Read(out this.critical_total);
            deserializer.Read(out this.health_score);
            deserializer.Read(out this.failed_node_names, deserializer.ReadLength());
            deserializer.Read(out this.degraded_node_names, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.total_nodes);
            serializer.Write(this.healthy_nodes);
            serializer.Write(this.degraded_nodes);
            serializer.Write(this.failed_nodes);
            serializer.Write(this.critical_healthy);
            serializer.Write(this.critical_total);
            serializer.Write(this.health_score);
            serializer.WriteLength(this.failed_node_names);
            serializer.Write(this.failed_node_names);
            serializer.WriteLength(this.degraded_node_names);
            serializer.Write(this.degraded_node_names);
        }

        public override string ToString()
        {
            return "FleetHealthMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\ntotal_nodes: " + total_nodes.ToString() +
            "\nhealthy_nodes: " + healthy_nodes.ToString() +
            "\ndegraded_nodes: " + degraded_nodes.ToString() +
            "\nfailed_nodes: " + failed_nodes.ToString() +
            "\ncritical_healthy: " + critical_healthy.ToString() +
            "\ncritical_total: " + critical_total.ToString() +
            "\nhealth_score: " + health_score.ToString() +
            "\nfailed_node_names: " + System.String.Join(", ", failed_node_names.ToList()) +
            "\ndegraded_node_names: " + System.String.Join(", ", degraded_node_names.ToList());
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
